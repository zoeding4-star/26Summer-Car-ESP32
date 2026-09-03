/**
 * 摄像头黑线循迹
 *
 * 控制对齐 main_line_tracking.c 的红外逻辑：
 *   - 看到竖线 → 直行，用竖线最低端偏角做小调/大调
 *   - 左侧或右侧出现大横带 / 折线 → 只记住转弯方向，不转
 *   - 看不到黑线 → 按记忆方向三轮同速同向原地自转
 *
 * 画面：大 y=车头近端。翻转用 CAM_FLIP_UD / CAM_FLIP_LR。
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_psram.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"

#include "usb_stream.h"
#include "tjpgd.h"

static const char *TAG = "CAM_LINE";

/* ==================== 电机 ==================== */
#define MOTOR_D_PWM     GPIO_NUM_14
#define MOTOR_D_IN1     GPIO_NUM_13
#define MOTOR_D_IN2     GPIO_NUM_12
#define MOTOR_A_PWM     GPIO_NUM_21
#define MOTOR_A_IN1     GPIO_NUM_46
#define MOTOR_A_IN2     GPIO_NUM_3
#define MOTOR_B_PWM     GPIO_NUM_15
#define MOTOR_B_IN1     GPIO_NUM_16
#define MOTOR_B_IN2     GPIO_NUM_17

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        16000
#define PWM_RESOL       8
#define MAX_SPEED       255
#define PWM_CAP         66

/* ==================== 摄像头 ==================== */
#define CAM_WIDTH           480
#define CAM_HEIGHT          320
#define CAM_FPS             30
#define JPEG_DSCALE         3       /* 1/8，把解码从 ~100ms 打到几十 ms */
#define JPEG_XFER_SIZE      (88 * 1024)
#define CAM_FLIP_UD         1
#define CAM_FLIP_LR         1

#define SCAN_Y_NEAR         320
#define SCAN_Y_FAR          220
#define SCAN_ROWS           5
#define ROI_X0_480          180
#define ROI_X1_480          300

#define LINE_THRESH_MIN     28
#define LINE_THRESH_MAX     140
#define MIN_LINE_W          1
#define MAX_LINE_W          18      /* 1/8 图上竖线很窄；更宽当横带 */
#define MAX_BLOBS           4
#define STEM_MAX_JUMP       8
#define BAR_FRAC            40      /* 一行黑像素占 ROI 百分比，视为横带 */
#define SIDE_RATIO          2.2f
#define MIN_SIDE_MASS       6
#define KINK_PX             3

/* 对齐红外：直行 + 两档角速度；自转三轮同速 */
#define BASE_SPEED          40.0f
#define OMEGA_MICRO         9.0f
#define OMEGA_MACRO         16.0f
#define ROT_MAX             18.0f
#define SPIN_PWM            26.0f
#define DEAD_PX_480         10
#define MACRO_PX_480        28
#define LOST_STOP_FRAMES    180

/* ==================== 类型 ==================== */
typedef struct {
    float D, A, B;
} MotorSpeed;

typedef enum {
    LAST_DIR_LEFT = 0,
    LAST_DIR_RIGHT
} LastDir;

typedef enum {
    MODE_FOLLOW = 0,
    MODE_SPIN
} DriveMode;

typedef enum {
    VIEW_NONE = 0,  /* 看不见黑线 */
    VIEW_STEM,      /* 有竖线，直行微调 */
    VIEW_BAR        /* 只有横带，还看见黑，先记方向再往前 */
} ViewType;

typedef struct {
    int left, right, cx, width, mass;
} Blob;

typedef struct {
    int n;
    Blob b[MAX_BLOBS];
    int black_n;
    int span;
    bool full_bar;
} RowScan;

typedef struct {
    ViewType type;
    int near_cx;
    int far_cx;
    int angle;       /* 最低端偏角：>0 底端偏右 */
    int offset;
    bool near_ok;
    bool has_black;
    bool turn_left;
    bool turn_right;
    int left_mass;
    int right_mass;
    int stem_n;
    int kink;
    int near_y;
    int far_y;
    int corner_x;
    int corner_y;
} Sight;

/* ==================== 全局 ==================== */
static LastDir g_last_dir = LAST_DIR_LEFT;
static DriveMode g_mode = MODE_FOLLOW;
static int g_lost_frames = 0;
static int g_decode_ms = 0;
static MotorSpeed g_last_wheels = {0, 0, 0};
static float g_last_vy = 0.0f;
static float g_last_om = 0.0f;

static SemaphoreHandle_t s_frame_mutex;
static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_dbg_mutex;
static uint8_t *s_jpeg;
static volatile uint32_t s_jpeg_len;
static uint8_t *s_gray;
static uint8_t *s_bin;
static uint8_t *s_dbg_gray;
static uint8_t *s_dbg_bin;
static int s_img_w = CAM_WIDTH;
static int s_img_h = CAM_HEIGHT;
static RowScan g_scan_rows[SCAN_ROWS];
static int g_scan_y[SCAN_ROWS];
static int g_poly_x[SCAN_ROWS];
static int g_poly_y[SCAN_ROWS];
static int g_poly_n;
static Sight g_dbg_path;
static int g_dbg_w;
static int g_dbg_h;
static char s_dbg_json[4096];
static int s_dbg_json_len;
static volatile bool s_dbg_ready;

/* ==================== 电机 ==================== */
static void motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MOTOR_D_PWM) | (1ULL << MOTOR_D_IN1) | (1ULL << MOTOR_D_IN2) |
                        (1ULL << MOTOR_A_PWM) | (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_PWM) | (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = PWM_RESOL,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
    };
    ch.channel = LEDC_CH_D; ch.gpio_num = MOTOR_D_PWM; ledc_channel_config(&ch);
    ch.channel = LEDC_CH_A; ch.gpio_num = MOTOR_A_PWM; ledc_channel_config(&ch);
    ch.channel = LEDC_CH_B; ch.gpio_num = MOTOR_B_PWM; ledc_channel_config(&ch);
}

static void set_motor(gpio_num_t in1, gpio_num_t in2, float speed, ledc_channel_t ch)
{
    int pwm = (int)roundf(speed);
    if (pwm > MAX_SPEED) pwm = MAX_SPEED;
    if (pwm < -MAX_SPEED) pwm = -MAX_SPEED;

    if (pwm > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, ch, pwm);
    } else if (pwm < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        ledc_set_duty(LEDC_MODE, ch, -pwm);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, ch, 0);
    }
    ledc_update_duty(LEDC_MODE, ch);
}

static void set_all_motors(const MotorSpeed *s)
{
    set_motor(MOTOR_D_IN1, MOTOR_D_IN2, s->D, LEDC_CH_D);
    set_motor(MOTOR_A_IN1, MOTOR_A_IN2, s->A, LEDC_CH_A);
    set_motor(MOTOR_B_IN1, MOTOR_B_IN2, s->B, LEDC_CH_B);
}

static void stop_motors(void)
{
    MotorSpeed z = {0, 0, 0};
    set_all_motors(&z);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 直行差速：B 只辅助一点点。自转不用这个。 */
static void drive(float vy, float omega)
{
    float fwd = 0.0f;
    if (vy > 1.0f) {
        fwd = clampf(vy, 0.0f, (float)PWM_CAP);
    }
    float rot = clampf(omega, -ROT_MAX, ROT_MAX);

    MotorSpeed s;
    s.D = clampf(fwd + rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.A = clampf(fwd - rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.B = clampf(rot * 0.70f, -(float)PWM_CAP, (float)PWM_CAP);

    g_last_vy = vy;
    g_last_om = rot;
    g_last_wheels = s;
    set_all_motors(&s);
}

/* 三轮同速原地转：右前 A 反向，D/B 同向 */
static void spin_in_place(bool left)
{
    float s = left ? -SPIN_PWM : SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    g_last_wheels = m;
    set_all_motors(&m);
}

/* ==================== 图像 ==================== */
static void *psram_alloc(size_t n)
{
    void *p = heap_caps_aligned_alloc(16, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_aligned_alloc(16, n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

static inline int map_x(int x, int w)
{
    if (x < 0) x = 0;
    if (x >= w) x = w - 1;
    return CAM_FLIP_LR ? (w - 1 - x) : x;
}

static inline int map_y(int y, int h)
{
    if (y < 0) y = 0;
    if (y >= h) y = h - 1;
    return CAM_FLIP_UD ? (h - 1 - y) : y;
}

static uint8_t luma_at(int x, int y)
{
    return s_gray[map_y(y, s_img_h) * s_img_w + map_x(x, s_img_w)];
}

typedef struct {
    const uint8_t *data;
    size_t len;
    size_t pos;
    uint8_t *gray;
    int stride;
} tjd_io_t;

static size_t tjd_in(JDEC *jd, uint8_t *buff, size_t nbyte)
{
    tjd_io_t *io = (tjd_io_t *)jd->device;
    if (io->pos >= io->len) {
        return 0;
    }
    if (io->pos + nbyte > io->len) {
        nbyte = io->len - io->pos;
    }
    if (buff) {
        memcpy(buff, io->data + io->pos, nbyte);
    }
    io->pos += nbyte;
    return nbyte;
}

static int tjd_out(JDEC *jd, void *bitmap, JRECT *rect)
{
    tjd_io_t *io = (tjd_io_t *)jd->device;
    const uint8_t *src = (const uint8_t *)bitmap;
    int bw = rect->right - rect->left + 1;
    for (int y = rect->top; y <= rect->bottom; y++) {
        memcpy(io->gray + (size_t)y * io->stride + rect->left, src, (size_t)bw);
        src += bw;
    }
    return 1;
}

static bool decode_mjpeg(const uint8_t *jpg, int len)
{
    static uint8_t pool[4096];
    JDEC jd;
    tjd_io_t io = {
        .data = jpg,
        .len = (size_t)len,
        .pos = 0,
        .gray = s_gray,
        .stride = CAM_WIDTH,
    };

    JRESULT r = jd_prepare(&jd, tjd_in, pool, sizeof(pool), &io);
    if (r != JDR_OK) {
        ESP_LOGW(TAG, "jd_prepare=%d", (int)r);
        return false;
    }
    if (jd.width == 0 || jd.height == 0 || jd.width > CAM_WIDTH || jd.height > CAM_HEIGHT) {
        return false;
    }

    int out_w = jd.width >> JPEG_DSCALE;
    int out_h = jd.height >> JPEG_DSCALE;
    if (out_w < 16 || out_h < 16) {
        return false;
    }
    s_img_w = out_w;
    s_img_h = out_h;
    io.stride = s_img_w;

    r = jd_decomp(&jd, tjd_out, JPEG_DSCALE);
    if (r != JDR_OK) {
        ESP_LOGW(TAG, "jd_decomp=%d", (int)r);
        return false;
    }
    return true;
}

static int scaled_px(int px480)
{
    int v = px480 * s_img_w / CAM_WIDTH;
    return (v < 1) ? 1 : v;
}

static int roi_x0(void)
{
    int x = scaled_px(ROI_X0_480);
    return (x < 0) ? 0 : x;
}

static int roi_x1(void)
{
    int x = scaled_px(ROI_X1_480);
    if (x >= s_img_w) {
        x = s_img_w - 1;
    }
    return x;
}

static int max_line_w(void)
{
    int w = scaled_px(MAX_LINE_W * CAM_WIDTH / 120);
    if (w < 4) {
        w = 4;
    }
    return w;
}

/* 只二值化扫描行，不整幅膨胀 */
static void scan_row(int y, RowScan *out)
{
    memset(out, 0, sizeof(*out));
    int x0 = roi_x0();
    int x1 = roi_x1();
    int span = x1 - x0 + 1;
    if (span < 4) {
        return;
    }
    out->span = span;

    int sum = 0;
    for (int x = x0; x <= x1; x++) {
        sum += luma_at(x, y);
    }
    int mean = sum / span;
    int th = mean - 18;
    if (th < LINE_THRESH_MIN) {
        th = LINE_THRESH_MIN;
    }
    if (th > LINE_THRESH_MAX) {
        th = LINE_THRESH_MAX;
    }

    int run0 = -1;
    int black_n = 0;
    for (int x = x0; x <= x1 + 1; x++) {
        bool on = false;
        if (x <= x1) {
            on = luma_at(x, y) < th;
            s_bin[y * s_img_w + x] = on ? 1 : 0;
            if (on) {
                black_n++;
            }
        }
        if (on) {
            if (run0 < 0) {
                run0 = x;
            }
        } else if (run0 >= 0) {
            int w = x - run0;
            if (w >= MIN_LINE_W && out->n < MAX_BLOBS) {
                Blob *b = &out->b[out->n++];
                b->left = run0;
                b->right = x - 1;
                b->width = w;
                b->cx = (run0 + x - 1) / 2;
                b->mass = w;
            }
            run0 = -1;
        }
    }
    out->black_n = black_n;
    out->full_bar = (black_n * 100 >= span * BAR_FRAC);
}

static int pick_stem_cx(const RowScan *row, int pred, int max_jump)
{
    int best = -1;
    int best_d = 10000;
    int cap = max_line_w();
    for (int i = 0; i < row->n; i++) {
        if (row->b[i].width > cap && !row->full_bar) {
            continue;
        }
        if (row->full_bar) {
            continue;
        }
        int d = abs(row->b[i].cx - pred);
        if (d < best_d && d <= max_jump) {
            best_d = d;
            best = row->b[i].cx;
        }
    }
    return best;
}

static void probe_sides(int cx, int y, int *left, int *right)
{
    int x0 = roi_x0();
    int x1 = roi_x1();
    int L = 0, R = 0;
    for (int x = x0; x <= x1; x++) {
        if (!s_bin[y * s_img_w + x]) {
            continue;
        }
        if (x < cx - 1) {
            L++;
        } else if (x > cx + 1) {
            R++;
        }
    }
    *left = L;
    *right = R;
}

static const char *view_name(ViewType t)
{
    switch (t) {
    case VIEW_STEM: return "STEM";
    case VIEW_BAR:  return "BAR";
    default:        return "NONE";
    }
}

static Sight look(void)
{
    Sight s;
    memset(&s, 0, sizeof(s));
    s.near_cx = -1;
    s.far_cx = -1;
    s.near_y = -1;
    s.far_y = -1;
    s.corner_x = -1;
    s.corner_y = -1;
    g_poly_n = 0;

    int h = s_img_h;
    int y_near = SCAN_Y_NEAR * h / CAM_HEIGHT;
    int y_far = SCAN_Y_FAR * h / CAM_HEIGHT;
    if (y_near >= h) {
        y_near = h - 1;
    }
    if (y_far < 0) {
        y_far = 0;
    }
    if (y_far >= y_near) {
        y_far = y_near - (SCAN_ROWS - 1);
        if (y_far < 0) {
            y_far = 0;
        }
    }

    int x0 = roi_x0();
    int x1 = roi_x1();
    for (int y = y_far; y <= y_near; y++) {
        memset(s_bin + y * s_img_w + x0, 0, (size_t)(x1 - x0 + 1));
    }

    for (int i = 0; i < SCAN_ROWS; i++) {
        int y = y_near - (y_near - y_far) * i / (SCAN_ROWS - 1);
        g_scan_y[i] = y;
        scan_row(y, &g_scan_rows[i]);
        if (g_scan_rows[i].black_n > 0) {
            s.has_black = true;
        }
    }

    int center = s_img_w / 2;
    int pred = center;
    int jump = scaled_px(STEM_MAX_JUMP * CAM_WIDTH / 120);
    if (jump < 4) {
        jump = 4;
    }
    int miss = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        int cx = pick_stem_cx(&g_scan_rows[i], pred, jump);
        if (cx < 0) {
            miss++;
            if (miss >= 2) {
                break;
            }
            continue;
        }
        miss = 0;
        g_poly_x[g_poly_n] = cx;
        g_poly_y[g_poly_n] = g_scan_y[i];
        g_poly_n++;
        pred = cx;
    }
    s.stem_n = g_poly_n;

    if (g_poly_n >= 1) {
        s.near_ok = true;
        s.near_cx = g_poly_x[0];
        s.near_y = g_poly_y[0];
        s.far_cx = g_poly_x[g_poly_n - 1];
        s.far_y = g_poly_y[g_poly_n - 1];
        s.offset = s.near_cx - center;
        if (g_poly_n >= 2) {
            s.angle = g_poly_x[0] - g_poly_x[1];
        } else {
            s.angle = s.offset;
        }
        s.type = VIEW_STEM;
    }

    int stem_cx = (g_poly_n >= 1) ? g_poly_x[0] : center;
    int Lall = 0, Rall = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        int L = 0, R = 0;
        probe_sides(stem_cx, g_scan_y[i], &L, &R);
        Lall += L;
        Rall += R;
        if (g_scan_rows[i].full_bar) {
            if (L > R * 1.2f) {
                s.turn_left = true;
            } else if (R > L * 1.2f) {
                s.turn_right = true;
            }
        }
        for (int k = 0; k < g_scan_rows[i].n; k++) {
            if (g_scan_rows[i].b[k].width >= max_line_w()) {
                if (g_scan_rows[i].b[k].cx < stem_cx) {
                    s.turn_left = true;
                } else if (g_scan_rows[i].b[k].cx > stem_cx) {
                    s.turn_right = true;
                }
            }
        }
    }
    s.left_mass = Lall;
    s.right_mass = Rall;
    if (Lall > (int)(Rall * SIDE_RATIO) && Lall >= MIN_SIDE_MASS) {
        s.turn_left = true;
    }
    if (Rall > (int)(Lall * SIDE_RATIO) && Rall >= MIN_SIDE_MASS) {
        s.turn_right = true;
    }

    /* 折线：最低端往上方向突变 */
    int kink = scaled_px(KINK_PX * CAM_WIDTH / 60);
    if (kink < 2) {
        kink = 2;
    }
    if (g_poly_n >= 3) {
        for (int i = 1; i < g_poly_n - 1; i++) {
            int a = g_poly_x[i] - g_poly_x[i - 1];
            int b = g_poly_x[i + 1] - g_poly_x[i];
            if (abs(a - b) >= kink && (a * b < 0 || abs(b) >= kink)) {
                s.kink = abs(a - b);
                s.corner_x = g_poly_x[i];
                s.corner_y = g_poly_y[i];
                if (b < 0 || (b == 0 && a > 0)) {
                    s.turn_left = true;
                } else {
                    s.turn_right = true;
                }
            }
        }
    }

    if (s.turn_left && s.turn_right) {
        /* 十字：两边都有，不记转弯 */
        s.turn_left = false;
        s.turn_right = false;
    }

    if (!s.near_ok && s.has_black) {
        s.type = VIEW_BAR;
        if (!s.turn_left && !s.turn_right) {
            if (Lall >= Rall) {
                s.turn_left = true;
            } else {
                s.turn_right = true;
            }
        }
    }
    if (!s.has_black) {
        s.type = VIEW_NONE;
    }
    return s;
}

static const char *debug_hint(const Sight *p)
{
    switch (p->type) {
    case VIEW_STEM:
        if (p->turn_left) {
            return "有竖线，左侧有横带或折线：记下左转，现在仍直行微调。";
        }
        if (p->turn_right) {
            return "有竖线，右侧有横带或折线：记下右转，现在仍直行微调。";
        }
        return "有竖线：按最低端偏角小调或大调，不转圈。";
    case VIEW_BAR:
        return "窗口里是横带、还看不见竖线：记下方向，继续往前，等丢线再自转。";
    default:
        return "看不见黑线：按记忆方向三轮同速同向原地转。";
    }
}

static void debug_copy_images(void)
{
    int w = s_img_w;
    int h = s_img_h;
    int x0 = roi_x0();
    int x1 = roi_x1();
    int y0 = g_scan_y[SCAN_ROWS - 1];
    int y1 = g_scan_y[0];
    if (y0 > y1) {
        int t = y0;
        y0 = y1;
        y1 = t;
    }
    for (int y = 0; y < h; y++) {
        uint8_t *dg = s_dbg_gray + y * w;
        const uint8_t *sg = s_gray + map_y(y, h) * w;
        if (CAM_FLIP_LR) {
            for (int x = 0; x < w; x++) {
                dg[x] = sg[w - 1 - x];
            }
        } else {
            memcpy(dg, sg, (size_t)w);
        }
        uint8_t *db = s_dbg_bin + y * w;
        memset(db, 40, (size_t)w);
        if (s_bin && y >= y0 && y <= y1) {
            for (int x = x0; x <= x1; x++) {
                db[x] = s_bin[y * w + x] ? 0 : 255;
            }
        }
    }
}

static void debug_update(const Sight *p)
{
    static int dbg_n;
    if ((++dbg_n & 7) != 0) {
        return;
    }
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin) {
        return;
    }
    if (xSemaphoreTake(s_dbg_mutex, 0) != pdTRUE) {
        return;
    }
    g_dbg_path = *p;
    g_dbg_w = s_img_w;
    g_dbg_h = s_img_h;
    debug_copy_images();

    int n = snprintf(s_dbg_json, sizeof(s_dbg_json),
                     "{\"type\":\"%s\",\"md\":%d,\"near\":%d,\"far\":%d,\"heading\":%d,"
                     "\"offset\":%d,\"L\":%d,\"R\":%d,\"vy\":%.0f,\"om\":%.0f,\"ms\":%d,"
                     "\"roi0\":%d,\"roi1\":%d,\"ny\":%d,\"fy\":%d,\"kx\":%d,\"ky\":%d,"
                     "\"stem\":%d,\"kink\":%d,\"rows\":%d,\"ldir\":\"%s\",\"hint\":\"%s\",\"scan_y\":[",
                     view_name(p->type), (int)g_mode, p->near_cx, p->far_cx, p->angle,
                     p->offset, p->left_mass, p->right_mass, g_last_vy, g_last_om, g_decode_ms,
                     roi_x0(), roi_x1(), p->near_y, p->far_y, p->corner_x, p->corner_y,
                     p->stem_n, p->kink, SCAN_ROWS,
                     (g_last_dir == LAST_DIR_LEFT) ? "L" : "R",
                     debug_hint(p));
    for (int i = 0; i < SCAN_ROWS && n > 0 && n < (int)sizeof(s_dbg_json) - 120; i++) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "%s%d",
                      (i ? "," : ""), g_scan_y[i]);
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 20) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "],\"blobs\":[");
    }
    int first = 1;
    for (int i = 0; i < SCAN_ROWS && n > 0 && n < (int)sizeof(s_dbg_json) - 80; i++) {
        for (int k = 0; k < g_scan_rows[i].n && n < (int)sizeof(s_dbg_json) - 80; k++) {
            n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n,
                          "%s{\"x\":%d,\"y\":%d,\"w\":%d}",
                          first ? "" : ",",
                          g_scan_rows[i].b[k].cx, g_scan_y[i], g_scan_rows[i].b[k].width);
            first = 0;
        }
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 20) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "],\"poly\":[");
    }
    first = 1;
    for (int i = 0; i < g_poly_n && n > 0 && n < (int)sizeof(s_dbg_json) - 40; i++) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n,
                      "%s{\"x\":%d,\"y\":%d}",
                      first ? "" : ",", g_poly_x[i], g_poly_y[i]);
        first = 0;
    }
    if (n > 0 && n < (int)sizeof(s_dbg_json) - 3) {
        n += snprintf(s_dbg_json + n, sizeof(s_dbg_json) - (size_t)n, "]}");
    }
    if (n < 0) {
        n = 0;
    }
    if (n >= (int)sizeof(s_dbg_json)) {
        n = (int)sizeof(s_dbg_json) - 1;
        s_dbg_json[n] = '\0';
    }
    s_dbg_json_len = n;
    s_dbg_ready = true;
    xSemaphoreGive(s_dbg_mutex);
}

static const char DBG_HTML[] =
    "<!DOCTYPE html><html><head><meta charset=utf-8><title>循迹画面</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#ddd;margin:16px;max-width:1100px}"
    ".row{display:flex;flex-wrap:wrap;gap:16px}"
    "figure{margin:0}"
    "canvas{background:#000;image-rendering:pixelated;width:min(46vw,480px);border:1px solid #444}"
    "figcaption{color:#aaa;margin:6px 0 0}"
    "pre{background:#1a1a1a;padding:10px;line-height:1.45;white-space:pre-wrap}"
    "ol{line-height:1.55;color:#ccc} li{margin:6px 0}</style></head><body>"
    "<h2>原画面 vs 用于判断的二值图</h2>"
    "<div class=row>"
    "<figure><canvas id=a width=60 height=40></canvas>"
    "<figcaption>左：原图（已翻转）。青框=检测窗口</figcaption></figure>"
    "<figure><canvas id=b width=60 height=40></canvas>"
    "<figcaption>右：逻辑图。白=地　黑=线　灰=不看。品红=竖线　绿=最低端</figcaption></figure>"
    "</div>"
    "<pre id=t>连接中...</pre>"
    "<h3>判断逻辑</h3>"
    "<ol>"
    "<li>只看青框。有竖线就直行，用绿点（竖线最低端）相对上一扫描行的偏角做微调。</li>"
    "<li>偏角小 → 小调；偏角大 → 大调。对标红外内侧灯 / 外侧灯。</li>"
    "<li>左侧或右侧出现大横带、折线：只记住左转或右转，现在不转圈。</li>"
    "<li>框里看不见黑线：按记忆方向三轮同速同向原地自转，直到竖线回来。</li>"
    "</ol>"
    "<script>"
    "function paint(cv,pix,w,h){cv.width=w;cv.height=h;const ctx=cv.getContext('2d');"
    "const im=ctx.createImageData(w,h);"
    "for(let i=0;i<w*h;i++){const v=pix[i]||0;im.data[i*4]=v;im.data[i*4+1]=v;im.data[i*4+2]=v;im.data[i*4+3]=255;}"
    "ctx.putImageData(im,0,0);return ctx;}"
    "let busy=false;"
    "async function tick(){"
    "if(busy)return;busy=true;"
    "try{"
    "const r=await fetch('/snap');"
    "if(!r.ok)throw new Error('HTTP '+r.status+' '+r.statusText);"
    "const buf=new Uint8Array(await r.arrayBuffer());"
    "const jl=buf[0]|buf[1]<<8;"
    "const inf=JSON.parse(new TextDecoder().decode(buf.subarray(2,2+jl)));"
    "const o=2+jl,w=buf[o]|buf[o+1]<<8,h=buf[o+2]|buf[o+3]<<8,n=w*h;"
    "const orig=buf.subarray(o+4,o+4+n),bin=buf.subarray(o+4+n,o+4+2*n);"
    "const c1=paint(document.getElementById('a'),orig,w,h);"
    "const c2=paint(document.getElementById('b'),bin,w,h);"
    "const r0=(inf.roi0|0),r1=(inf.roi1||w);"
    "const yTop=inf.scan_y[inf.scan_y.length-1]||0,yBot=inf.scan_y[0]||h;"
    "c1.strokeStyle='#0cf';c1.strokeRect(r0+0.5,yTop+0.5,Math.max(2,r1-r0),yBot-yTop);"
    "c2.strokeStyle='#0cf';c2.strokeRect(r0+0.5,yTop+0.5,Math.max(2,r1-r0),yBot-yTop);"
    "c2.strokeStyle='#cc0';"
    "for(const y of inf.scan_y){c2.beginPath();c2.moveTo(r0,y+0.5);c2.lineTo(r1,y+0.5);c2.stroke();}"
    "c2.strokeStyle='#f0f';c2.lineWidth=2;c2.beginPath();"
    "if(inf.poly&&inf.poly.length){inf.poly.forEach((p,i)=>{i?c2.lineTo(p.x,p.y):c2.moveTo(p.x,p.y);});c2.stroke();}"
    "c2.lineWidth=1;c2.strokeStyle='#f44';"
    "for(const b of inf.blobs){c2.strokeRect(b.x-b.w/2,b.y-1,Math.max(2,b.w),3);}"
    "function dot(ctx,x,y,c){if(x<0||y<0)return;ctx.fillStyle=c;ctx.beginPath();ctx.arc(x,y,3.5,0,6.28);ctx.fill();}"
    "dot(c2,inf.near,inf.ny,'#0f0');dot(c2,inf.far,inf.fy,'#0ff');dot(c2,inf.kx,inf.ky,'#fa0');"
    "const md=['直行 FOLLOW','自转 SPIN'][inf.md]||inf.md;"
    "document.getElementById('t').textContent="
    "'判定：'+inf.type+'    模式：'+md+'    记忆方向：'+inf.ldir+"
    "'\\n最低端(绿) n='+inf.near+' 偏角 heading='+inf.heading+' 偏移 offset='+inf.offset+"
    "'\\nstem='+inf.stem+'  kink='+inf.kink+'  L/R='+inf.L+'/'+inf.R+"
    "'\\nvy='+inf.vy+' om='+inf.om+'  解码 '+inf.ms+'ms'"
    "+'\\n\\n'+inf.hint;"
    "}catch(e){document.getElementById('t').textContent='等待画面 '+e;}"
    "busy=false;}"
    "setInterval(tick,500);tick();"
    "</script></body></html>";

static void dbg_http_hdr(httpd_req_t *req, const char *type)
{
    httpd_resp_set_type(req, type);
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
}

static esp_err_t dbg_index_handler(httpd_req_t *req)
{
    dbg_http_hdr(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, DBG_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t dbg_ping_handler(httpd_req_t *req)
{
    dbg_http_hdr(req, "text/plain");
    return httpd_resp_send(req, "ok", 2);
}

static bool dbg_copy_locked(char *json, int *json_len, uint8_t *gray, uint8_t *bin, int *w, int *h)
{
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin || !s_dbg_ready) {
        return false;
    }
    if (xSemaphoreTake(s_dbg_mutex, pdMS_TO_TICKS(40)) != pdTRUE) {
        return false;
    }
    *json_len = s_dbg_json_len;
    if (*json_len < 0) {
        *json_len = 0;
    }
    if (*json_len > (int)sizeof(s_dbg_json)) {
        *json_len = (int)sizeof(s_dbg_json);
    }
    memcpy(json, s_dbg_json, (size_t)*json_len);
    *w = g_dbg_w;
    *h = g_dbg_h;
    if (*w > 0 && *h > 0) {
        size_t n = (size_t)(*w) * (size_t)(*h);
        if (gray) {
            memcpy(gray, s_dbg_gray, n);
        }
        if (bin) {
            memcpy(bin, s_dbg_bin, n);
        }
    }
    xSemaphoreGive(s_dbg_mutex);
    return *w > 0 && *h > 0 && *json_len > 2;
}

static esp_err_t dbg_snap_handler(httpd_req_t *req)
{
    static char json[4096];
    static uint8_t *gray;
    static uint8_t *bin;
    if (!gray) {
        gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    }
    if (!bin) {
        bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    }
    int jl = 0, w = 0, h = 0;
    if (!gray || !bin || !dbg_copy_locked(json, &jl, gray, bin, &w, &h)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        dbg_http_hdr(req, "text/plain");
        return httpd_resp_send(req, "wait", 4);
    }

    size_t pix = (size_t)w * (size_t)h;
    size_t total = 6 + (size_t)jl + pix * 2;
    uint8_t *pkt = (uint8_t *)psram_alloc(total);
    if (!pkt) {
        return httpd_resp_send_500(req);
    }
    pkt[0] = (uint8_t)(jl & 0xff);
    pkt[1] = (uint8_t)((jl >> 8) & 0xff);
    memcpy(pkt + 2, json, (size_t)jl);
    pkt[2 + jl] = (uint8_t)(w & 0xff);
    pkt[3 + jl] = (uint8_t)((w >> 8) & 0xff);
    pkt[4 + jl] = (uint8_t)(h & 0xff);
    pkt[5 + jl] = (uint8_t)((h >> 8) & 0xff);
    memcpy(pkt + 6 + jl, gray, pix);
    memcpy(pkt + 6 + jl + pix, bin, pix);

    dbg_http_hdr(req, "application/octet-stream");
    esp_err_t r = httpd_resp_send(req, (const char *)pkt, (ssize_t)total);
    free(pkt);
    return r;
}

static void wifi_debug_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t el = esp_event_loop_create_default();
    if (el != ESP_OK && el != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(el);
    }
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = { 0 };
    memcpy(wifi_config.ap.ssid, "CAM_LINE", 8);
    wifi_config.ap.ssid_len = 8;
    wifi_config.ap.channel = 1;
    memcpy(wifi_config.ap.password, "12345678", 8);
    wifi_config.ap.max_connection = 2;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.core_id = 1;
    config.max_open_sockets = 4;
    config.recv_wait_timeout = 10;
    config.send_wait_timeout = 10;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 启动失败");
        return;
    }
    httpd_uri_t u0 = { .uri = "/", .method = HTTP_GET, .handler = dbg_index_handler };
    httpd_uri_t u1 = { .uri = "/ping", .method = HTTP_GET, .handler = dbg_ping_handler };
    httpd_uri_t u2 = { .uri = "/snap", .method = HTTP_GET, .handler = dbg_snap_handler };
    httpd_register_uri_handler(server, &u0);
    httpd_register_uri_handler(server, &u1);
    httpd_register_uri_handler(server, &u2);
    ESP_LOGI(TAG, "调试页: 连 WiFi CAM_LINE 密码 12345678，浏览器打开 http://192.168.4.1/");
}

static void follow_stem(const Sight *p)
{
    int dead = scaled_px(DEAD_PX_480);
    int macro = scaled_px(MACRO_PX_480);
    int err = p->angle;
    if (abs(err) <= dead && abs(p->offset) > macro) {
        err = p->offset;
    }
    if (abs(err) <= dead) {
        drive(BASE_SPEED, 0.0f);
        return;
    }
    float om = (abs(err) <= macro) ? OMEGA_MICRO : OMEGA_MACRO;
    if (err < 0) {
        om = -om;
    }
    drive(BASE_SPEED, om);
}

static void control_once(void)
{
    Sight p = look();

    if (p.turn_left) {
        g_last_dir = LAST_DIR_LEFT;
    } else if (p.turn_right) {
        g_last_dir = LAST_DIR_RIGHT;
    }

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG, "%s stem=%d ang=%d off=%d LR=%d/%d turn=%s%s md=%d vy=%.0f om=%.0f %dms",
                 view_name(p.type), p.stem_n, p.angle, p.offset, p.left_mass, p.right_mass,
                 p.turn_left ? "L" : "", p.turn_right ? "R" : "",
                 (int)g_mode, g_last_vy, g_last_om, g_decode_ms);
        last_log = now;
    }

    if (p.type == VIEW_NONE) {
        g_lost_frames++;
        if (g_lost_frames > LOST_STOP_FRAMES) {
            stop_motors();
            ESP_LOGE(TAG, "连续丢线，停车保护");
            debug_update(&p);
            return;
        }
        g_mode = MODE_SPIN;
        spin_in_place(g_last_dir == LAST_DIR_LEFT);
        debug_update(&p);
        return;
    }

    g_lost_frames = 0;
    g_mode = MODE_FOLLOW;
    if (p.type == VIEW_STEM) {
        follow_stem(&p);
    } else {
        /* 只有横带：还看得见黑，先往前开，等丢线再转 */
        drive(BASE_SPEED, 0.0f);
    }
    debug_update(&p);
}

static void camera_frame_cb(uvc_frame_t *frame, void *ptr)
{
    (void)ptr;
    if (!frame || !frame->data || frame->data_bytes == 0 || !s_frame_mutex) {
        return;
    }
    if (frame->data_bytes > JPEG_XFER_SIZE) {
        return;
    }
    if (xSemaphoreTake(s_frame_mutex, 0) == pdTRUE) {
        memcpy(s_jpeg, frame->data, frame->data_bytes);
        s_jpeg_len = frame->data_bytes;
        xSemaphoreGive(s_frame_mutex);
        xSemaphoreGive(s_frame_ready);
    }
}

static bool camera_start(void)
{
    uint8_t *xfer_a = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    uint8_t *xfer_b = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    uint8_t *frame_buf = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    s_jpeg = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    s_gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);

    if (!xfer_a || !xfer_b || !frame_buf || !s_jpeg || !s_gray || !s_bin) {
        ESP_LOGE(TAG, "PSRAM 分配失败");
        return false;
    }

    uvc_config_t cfg = {
        .frame_width = CAM_WIDTH,
        .frame_height = CAM_HEIGHT,
        .frame_index = 4,
        .frame_interval = FPS2INTERVAL(CAM_FPS),
        .xfer_buffer_size = JPEG_XFER_SIZE,
        .xfer_buffer_a = xfer_a,
        .xfer_buffer_b = xfer_b,
        .frame_buffer_size = JPEG_XFER_SIZE,
        .frame_buffer = frame_buf,
        .frame_cb = camera_frame_cb,
        .frame_cb_arg = NULL,
        .format = UVC_FORMAT_MJPEG,
    };

    ESP_LOGI(TAG, "UVC MJPEG %dx%d @ %d fps", CAM_WIDTH, CAM_HEIGHT, CAM_FPS);
    if (uvc_streaming_config(&cfg) != ESP_OK) {
        ESP_LOGE(TAG, "UVC 配置失败");
        return false;
    }
    if (usb_streaming_start() != ESP_OK) {
        ESP_LOGE(TAG, "USB 启动失败");
        return false;
    }
    return true;
}

static void line_task(void *arg)
{
    (void)arg;
    uint8_t *local_jpg = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    if (!local_jpg) {
        ESP_LOGE(TAG, "任务 JPEG 缓冲分配失败");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "循迹任务启动");
    while (1) {
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(150)) != pdTRUE) {
            g_lost_frames++;
            if (g_lost_frames > LOST_STOP_FRAMES) {
                stop_motors();
            }
            continue;
        }
        while (xSemaphoreTake(s_frame_ready, 0) == pdTRUE) {
        }

        uint32_t len = 0;
        if (xSemaphoreTake(s_frame_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            len = s_jpeg_len;
            if (len > 0 && len <= JPEG_XFER_SIZE) {
                memcpy(local_jpg, s_jpeg, len);
            }
            xSemaphoreGive(s_frame_mutex);
        }
        if (len == 0) {
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        if (!decode_mjpeg(local_jpg, (int)len)) {
            ESP_LOGW(TAG, "JPEG 解码失败 len=%u", (unsigned)len);
            continue;
        }
        g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        control_once();
    }
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    if (esp_psram_is_initialized()) {
        ESP_LOGI(TAG, "PSRAM %d KB", (int)(esp_psram_get_size() / 1024));
    } else {
        ESP_LOGW(TAG, "PSRAM 未启用，解码缓冲可能不足");
    }

    s_frame_mutex = xSemaphoreCreateMutex();
    s_frame_ready = xSemaphoreCreateBinary();
    s_dbg_mutex = xSemaphoreCreateMutex();
    s_dbg_gray = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_dbg_bin = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    if (!s_dbg_gray || !s_dbg_bin) {
        ESP_LOGW(TAG, "调试画面缓冲分配失败，网页将没有图像");
    }

    motor_init();
    stop_motors();
    wifi_debug_start();

    if (!camera_start()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    xTaskCreatePinnedToCore(line_task, "line_follow", 12288, NULL, 6, NULL, 0);
}
