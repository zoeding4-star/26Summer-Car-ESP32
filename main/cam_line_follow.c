/**
 * 摄像头黑线循迹（直线 / 锐角 / 直角，并区分左转右转）
 *
 * 硬件参考本工程：
 *   - 电机引脚与三轮全向运动学：cam_motion.c / main_final.c
 *   - UVC MJPEG 取流：camera.c（usb_stream，480x320）
 *
 * 画面约定：摄像头前视俯拍地面，y=0 在画面上方（远端），画面底部是车头近端。
 * 若实际装反，把 CAM_FLIP_UD / CAM_FLIP_LR 改为 1。
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

#include "usb_stream.h"
#include "tjpgd.h"

static const char *TAG = "CAM_LINE";

/* ==================== 电机引脚（与 cam_motion.c 一致） ==================== */
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
#define PWM_FREQ        16000   /* 提高载波，静摩擦下力矩更连续 */
#define PWM_RESOL       8
#define MAX_SPEED       255
#define PWM_CAP         66      /* 轮速硬上限；直行不再默认拉满 */

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f

#define FWD_SCALE_D     1.00f
#define FWD_SCALE_A     1.00f
#define FWD_SCALE_B     1.00f
#define ROT_SCALE_D     1.00f
#define ROT_SCALE_A     1.00f
#define ROT_SCALE_B     1.00f

/* ==================== 摄像头 ==================== */
#define CAM_WIDTH           480
#define CAM_HEIGHT          320
#define CAM_FPS             15
#define JPEG_DSCALE         2       /* 1/4 分辨率，优先把控制周期打上去 */
#define JPEG_XFER_SIZE      (88 * 1024)
#define CAM_FLIP_UD         1       /* 画面上下颠倒时改 1 */
#define CAM_FLIP_LR         1       /* 画面左右镜像时改 1 */

/* 逻辑坐标：大 y=车头近端，小 y=画面上方远预瞄。最后几行尽量靠近地平线，才能提前看到弯 */
#define SCAN_ROWS           10
static const int SCAN_Y[SCAN_ROWS] = { 292, 250, 210, 170, 135, 100, 72, 48, 28, 12 };
#define NEAR_IDX            0
#define MID_IDX             4
#define FAR_IDX             9
#define LOOKAHEAD_FROM      6

#define LINE_THRESH_MIN     28
#define LINE_THRESH_MAX     140
#define MIN_LINE_W          5
#define MAX_LINE_W          70
#define HORIZ_BAR_W         95      /* 超过此宽度视为横向横杠（直角特征） */
#define MAX_BLOBS           4

/* vy 直接就是前轮前进 PWM。66 冲出弯道，远点丢失时更要收油 */
#define BASE_SPEED          56.0f
#define CAUTION_SPEED       44.0f   /* 远点看不到时 */
#define CURVE_SPEED         36.0f
#define ACUTE_SPEED         26.0f
#define CORNER_APPROACH     24.0f
#define KP                  0.14f
#define KD                  0.06f
#define KP_ACUTE            0.22f
#define KD_ACUTE            0.08f
#define DEADBAND            4.0f
#define MAX_OMEGA_PD        14.0f
#define MAX_OMEGA_ACUTE     20.0f
#define OMEGA_CORNER        22.0f
#define OMEGA_SEARCH        20.0f
#define ROT_MAX             22.0f
#define SIDE_RATIO          2.4f
#define MIN_SIDE_MASS       24
#define LOST_STOP_FRAMES    40
#define CORNER_SPIN_TIMEOUT_MS  1200
#define CORNER_CONFIRM_FRAMES   4
#define ERR_FILTER          0.50f
#define HOLD_TURN_FRAMES        6
#define DIR_LOCK_MS             350
#define SLOW_HOLD_FRAMES        16  /* 见过弯后保持低速，避免判回 STRAIGHT 又踩满 */

/* ==================== 类型 ==================== */
typedef struct {
    float D, A, B;
} MotorSpeed;

typedef struct {
    float vx, vy, omega;
} Velocity;

typedef enum {
    LAST_DIR_LEFT = 0,
    LAST_DIR_RIGHT
} LastDir;

typedef enum {
    PATH_LOST = 0,
    PATH_STRAIGHT,
    PATH_CURVE_LEFT,
    PATH_CURVE_RIGHT,
    PATH_ACUTE_LEFT,
    PATH_ACUTE_RIGHT,
    PATH_RIGHT_ANGLE_LEFT,
    PATH_RIGHT_ANGLE_RIGHT,
    PATH_CROSS
} PathType;

typedef enum {
    MODE_FOLLOW = 0,
    MODE_CORNER,
    MODE_SEARCH
} DriveMode;

typedef struct {
    int left, right, cx, width, mass;
} Blob;

typedef struct {
    int n;
    Blob b[MAX_BLOBS];
    int main_cx;
    int main_w;
    bool valid;
    bool wide_bar;
} RowScan;

typedef struct {
    PathType type;
    int near_cx;
    int far_cx;
    int heading;
    int offset;
    bool near_ok;
    bool far_ok;
    int left_mass;
    int right_mass;
} PathInfo;

/* ==================== 全局 ==================== */
static LastDir g_last_dir = LAST_DIR_LEFT;
static DriveMode g_mode = MODE_FOLLOW;
static PathType g_corner_dir = PATH_RIGHT_ANGLE_LEFT;
static int64_t g_corner_t0 = 0;
static int g_lost_frames = 0;
static int g_corner_hits = 0;
static PathType g_hold_turn = PATH_LOST;
static int g_hold_frames = 0;
static LastDir g_dir_lock = LAST_DIR_LEFT;
static int64_t g_dir_lock_until = 0;
static int g_slow_frames = 0;
static int g_wrong_corner = 0;
static float g_last_err = 0.0f;
static int g_decode_ms = 0;
static float g_err_filt = 0.0f;
static MotorSpeed g_last_wheels = {0, 0, 0};
static float g_last_vy = 0.0f;
static float g_last_om = 0.0f;

static SemaphoreHandle_t s_frame_mutex;
static SemaphoreHandle_t s_frame_ready;
static uint8_t *s_jpeg;
static volatile uint32_t s_jpeg_len;
static uint8_t *s_gray;
static int s_img_w = CAM_WIDTH;
static int s_img_h = CAM_HEIGHT;

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

static MotorSpeed __attribute__((unused)) inverse_kinematics(const Velocity *vel)
{
    MotorSpeed w;
    float L = WHEEL_DISTANCE;
    w.D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    w.A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    w.B =  vel->vx + L * vel->omega;
    return w;
}

static float lock_omega(float om)
{
    if (esp_timer_get_time() >= g_dir_lock_until) {
        return om;
    }
    /* 锁定期内只允许原先那个方向的差速，反向指令直接抹掉 */
    if (g_dir_lock == LAST_DIR_LEFT && om > 0.0f) {
        return 0.0f;
    }
    if (g_dir_lock == LAST_DIR_RIGHT && om < 0.0f) {
        return 0.0f;
    }
    return om;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* vy 即前进 PWM，转向只用差速，禁止再按峰值拉满 */
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
    s.B = clampf(rot * 0.45f, -(float)PWM_CAP, (float)PWM_CAP);

    g_last_vy = vy;
    g_last_om = rot;
    g_last_wheels = s;
    set_all_motors(&s);
}

/* ==================== 图像访问 ==================== */
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
    int w = s_img_w;
    int h = s_img_h;
    return s_gray[map_y(y, h) * w + map_x(x, w)];
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
    /* 让出 CPU，避免 IDLE0 看门狗，同时提高后续控制节奏 */
    if ((rect->top & 0x0F) == 0) {
        vTaskDelay(0);
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
    if (out_w < 20 || out_h < 20) {
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
    return (v < 2) ? 2 : v;
}

static int horiz_bar_w(void)
{
    int w = s_img_w / 3;
    return (w < 28) ? 28 : w;
}

static int max_line_w(void)
{
    int w = scaled_px(MAX_LINE_W);
    return (w < 16) ? 16 : w;
}
static int adaptive_thresh(void)
{
    int w = s_img_w;
    uint32_t sum = 0;
    int n = 0;
    int min_l = 255;
    for (int i = 0; i < SCAN_ROWS; i++) {
        int y = SCAN_Y[i] * s_img_h / CAM_HEIGHT;
        for (int x = 8; x < w - 8; x += 8) {
            int v = luma_at(x, y);
            sum += (uint32_t)v;
            n++;
            if (v < min_l) {
                min_l = v;
            }
        }
    }
    if (n == 0) {
        return 90;
    }
    int mean = (int)(sum / (uint32_t)n);
    int span = mean - min_l;
    int th;
    if (span < 18) {
        /* 对比度差：只吃明显比均值暗的点，避免把深色地板当线 */
        th = mean - 12;
    } else {
        /* 贴着最暗的线，不要把偏暗的地面整片算进去 */
        th = min_l + span * 2 / 5;
    }
    if (th < LINE_THRESH_MIN) th = LINE_THRESH_MIN;
    if (th > LINE_THRESH_MAX) th = LINE_THRESH_MAX;
    return th;
}

static void scan_row(int y, int thresh, int min_w, RowScan *out)
{
    memset(out, 0, sizeof(*out));
    int w = s_img_w;
    int x0 = 4;
    int x1 = w - 5;
    int span = x1 - x0 + 1;
    int black_n = 0;
    for (int x = x0; x <= x1; x++) {
        if (luma_at(x, y) < thresh) {
            black_n++;
        }
    }
    /* 整行大面积偏暗 = 地板/阴影，不是线 */
    if (span > 0 && black_n * 100 > span * 48) {
        return;
    }

    int run_l = -1;
    for (int x = x0; x <= x1 + 1; x++) {
        bool black = (x <= x1) && (luma_at(x, y) < thresh);
        if (black && run_l < 0) {
            run_l = x;
        } else if (!black && run_l >= 0) {
            int run_r = x - 1;
            int width = run_r - run_l + 1;
            if (width >= min_w && out->n < MAX_BLOBS) {
                int mass = 0, sx = 0;
                for (int k = run_l; k <= run_r; k++) {
                    if (luma_at(k, y) < thresh) {
                        mass++;
                        sx += k;
                    }
                }
                if (mass >= min_w) {
                    Blob *b = &out->b[out->n++];
                    b->left = run_l;
                    b->right = run_r;
                    b->width = width;
                    b->mass = mass;
                    b->cx = sx / mass;
                    if (width >= horiz_bar_w()) {
                        out->wide_bar = true;
                    }
                }
            }
            run_l = -1;
        }
    }

    if (out->n > 0) {
        out->valid = true;
        /* 先选最接近画面中心的 blob 作为主线，后续会按路径跟踪修正 */
        int cx0 = w / 2;
        int best = 0;
        int best_d = 9999;
        for (int i = 0; i < out->n; i++) {
            int d = abs(out->b[i].cx - cx0);
            if (d < best_d && out->b[i].width <= max_line_w() * 2) {
                best_d = d;
                best = i;
            }
        }
        /* 若存在超宽横杠，优先记宽度，主 cx 仍用最接近中心且不太宽的段 */
        out->main_cx = out->b[best].cx;
        out->main_w = out->b[best].width;
    }
}

static int pick_main_cx(const RowScan *row, int pred_cx)
{
    if (!row->valid) {
        return -1;
    }
    int best = 0;
    int best_d = 9999;
    for (int i = 0; i < row->n; i++) {
        int d = abs(row->b[i].cx - pred_cx);
        /* 直角横杠上不要把整条杠的中点当成主线 */
        if (row->b[i].width > horiz_bar_w()) {
            continue;
        }
        if (d < best_d) {
            best_d = d;
            best = i;
        }
    }
    if (best_d == 9999) {
        return row->b[0].cx;
    }
    return row->b[best].cx;
}

static int side_mass(int y, int thresh, bool left)
{
    int w = s_img_w;
    int x_a, x_b;
    if (left) {
        x_a = 6;
        x_b = w / 3;
    } else {
        x_a = w * 2 / 3;
        x_b = w - 7;
    }
    /* 只统计成段的黑线，不把深色地面散点算成分叉 */
    int m = 0;
    int run = 0;
    int min_w = scaled_px(MIN_LINE_W);
    int max_w = max_line_w() * 2;
    for (int x = x_a; x <= x_b + 1; x++) {
        bool black = (x <= x_b) && (luma_at(x, y) < thresh);
        if (black) {
            run++;
        } else if (run > 0) {
            if (run >= min_w && run <= max_w) {
                m += run;
            }
            run = 0;
        }
    }
    return m;
}

static const char *path_name(PathType t)
{
    switch (t) {
    case PATH_STRAIGHT: return "STRAIGHT";
    case PATH_CURVE_LEFT: return "CURVE_L";
    case PATH_CURVE_RIGHT: return "CURVE_R";
    case PATH_ACUTE_LEFT: return "ACUTE_L";
    case PATH_ACUTE_RIGHT: return "ACUTE_R";
    case PATH_RIGHT_ANGLE_LEFT: return "90_L";
    case PATH_RIGHT_ANGLE_RIGHT: return "90_R";
    case PATH_CROSS: return "CROSS";
    default: return "LOST";
    }
}

/**
 * 分类要点：
 * - 直线：近远端都有线，质心差小
 * - 缓弯：连续斜线，heading 中等
 * - 锐角：连续斜线，heading 很大（远端线已明显甩到一侧）
 * - 直角：近端仍有竖线，远端中轴丢线，中部出现横杠或单侧分叉
 * 左右：看 heading 符号，或左右翼黑色像素量
 */
static PathInfo classify_path(void)
{
    PathInfo info = {0};
    int thresh = adaptive_thresh();
    int h = s_img_h;
    int w = s_img_w;
    int cx_img = w / 2;

    RowScan rows[SCAN_ROWS];
    for (int i = 0; i < SCAN_ROWS; i++) {
        int y = SCAN_Y[i] * h / CAM_HEIGHT;
        int min_w = scaled_px((i < 3) ? MIN_LINE_W + 2 : MIN_LINE_W);
        scan_row(y, thresh, min_w, &rows[i]);
    }

    /* 从近到远跟踪主路径 */
    int pred = cx_img;
    int tracked[SCAN_ROWS];
    for (int i = 0; i < SCAN_ROWS; i++) {
        tracked[i] = pick_main_cx(&rows[i], pred);
        if (tracked[i] >= 0) {
            pred = tracked[i];
        }
    }

    /* 远端横杠不算竖线预瞄；同一行里若还有窄 blob，仍可当主线 */
    int far_center_ok = 0;
    int far_line_n = 0;
    int far_cx_best = -1;
    for (int i = FAR_IDX; i >= LOOKAHEAD_FROM; i--) {
        if (tracked[i] < 0) {
            continue;
        }
        int tw = max_line_w() + 1;
        for (int k = 0; k < rows[i].n; k++) {
            if (rows[i].b[k].cx == tracked[i]) {
                tw = rows[i].b[k].width;
                break;
            }
        }
        if (tw > max_line_w()) {
            continue;
        }
        int margin = s_img_w / 8;
        if (tracked[i] < margin || tracked[i] > s_img_w - 1 - margin) {
            continue;
        }
        far_line_n++;
        if (far_cx_best < 0) {
            far_cx_best = tracked[i];
        }
        if (abs(tracked[i] - cx_img) < w / 5) {
            far_center_ok++;
        }
    }
    bool far_stem = far_line_n >= 1;

    info.near_cx = tracked[NEAR_IDX];
    info.near_ok = info.near_cx >= 0;
    info.far_cx = far_cx_best;
    info.far_ok = far_stem && far_cx_best >= 0;
    info.offset = info.near_ok ? (info.near_cx - cx_img) : 0;
    info.heading = (info.near_ok && info.far_ok) ? (info.far_cx - info.near_cx) : 0;

    int mid_y = SCAN_Y[MID_IDX] * h / CAM_HEIGHT;
    int near_y = SCAN_Y[NEAR_IDX] * h / CAM_HEIGHT;
    int far_y = SCAN_Y[FAR_IDX] * h / CAM_HEIGHT;
    int far2_y = SCAN_Y[FAR_IDX - 1] * h / CAM_HEIGHT;
    info.left_mass = side_mass(mid_y, thresh, true) + side_mass(near_y, thresh, true)
                   + side_mass(far_y, thresh, true) + side_mass(far2_y, thresh, true);
    info.right_mass = side_mass(mid_y, thresh, false) + side_mass(near_y, thresh, false)
                    + side_mass(far_y, thresh, false) + side_mass(far2_y, thresh, false);

    int valid_n = 0;
    int wide_n = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        if (rows[i].valid) {
            valid_n++;
        }
        if (rows[i].wide_bar) {
            wide_n++;
        }
    }

    bool far_lost = !far_stem;
    int min_side = scaled_px(MIN_SIDE_MASS);
    bool both_branch = info.left_mass > min_side && info.right_mass > min_side &&
                       info.left_mass * 2 > info.right_mass && info.right_mass * 2 > info.left_mass;

    if (valid_n == 0 && !info.near_ok) {
        info.type = PATH_LOST;
        return info;
    }

    /* 十字：远端中轴仍有线，且两侧都有分叉 → 直行穿过 */
    if (both_branch && far_center_ok >= 1 && info.near_ok && abs(info.heading) < (w / 8)) {
        info.type = PATH_CROSS;
        return info;
    }

    /*
     * 直角：近端明显偏到一侧 + 看到横杠 + 远端丢线。
     * 方向跟近端质心，不要用左右翼黑像素——新赛道地板偏暗时右侧会一直虚高，
     * 日志里 CURVE_L 紧接着被判成 90_R 然后反方向转。
     */
    bool near_centered = info.near_ok && abs(info.offset) < (w / 6);
    if (info.near_ok && !near_centered && wide_n > 0 && far_lost) {
        if (info.offset < 0) {
            info.type = PATH_RIGHT_ANGLE_LEFT;
        } else {
            info.type = PATH_RIGHT_ANGLE_RIGHT;
        }
        return info;
    }

    int acute_px = s_img_w / 3;
    int curve_px = s_img_w / 10;

    if (info.near_ok && info.far_ok) {
        int ah = abs(info.heading);
        if (ah >= acute_px) {
            info.type = (info.heading < 0) ? PATH_ACUTE_LEFT : PATH_ACUTE_RIGHT;
        } else if (ah >= curve_px) {
            info.type = (info.heading < 0) ? PATH_CURVE_LEFT : PATH_CURVE_RIGHT;
        } else {
            info.type = PATH_STRAIGHT;
        }
        return info;
    }

    /* 只有近端：按偏移判断，不要把丢远点误判成锐角/直角 */
    if (info.near_ok) {
        int off = abs(info.offset);
        if (off > (w / 8)) {
            info.type = (info.offset < 0) ? PATH_CURVE_LEFT : PATH_CURVE_RIGHT;
        } else {
            info.type = PATH_STRAIGHT;
        }
        return info;
    }

    info.type = PATH_LOST;
    return info;
}

static float pd_omega(int error_px, float kp, float kd, float max_w)
{
    float err = (float)error_px;
    g_err_filt = ERR_FILTER * g_err_filt + (1.0f - ERR_FILTER) * err;
    err = g_err_filt;
    if (fabsf(err) < DEADBAND) {
        err = 0.0f;
    }
    float d = err - g_last_err;
    g_last_err = err;
    float w = kp * err + kd * d;
    if (w > max_w) w = max_w;
    if (w < -max_w) w = -max_w;
    return w;
}

static void remember_dir(const PathInfo *p)
{
    switch (p->type) {
    case PATH_CURVE_LEFT:
    case PATH_ACUTE_LEFT:
    case PATH_RIGHT_ANGLE_LEFT:
        g_last_dir = LAST_DIR_LEFT;
        break;
    case PATH_CURVE_RIGHT:
    case PATH_ACUTE_RIGHT:
    case PATH_RIGHT_ANGLE_RIGHT:
        g_last_dir = LAST_DIR_RIGHT;
        break;
    default:
        if (p->near_ok) {
            g_last_dir = (p->near_cx < s_img_w / 2) ? LAST_DIR_LEFT : LAST_DIR_RIGHT;
        }
        break;
    }
}

static float fused_error(const PathInfo *p)
{
    if (p->near_ok && p->far_ok) {
        return 0.40f * (float)p->offset + 0.60f * (float)p->heading;
    }
    if (p->near_ok) {
        return (float)p->offset;
    }
    return (g_last_dir == LAST_DIR_LEFT) ? -MAX_OMEGA_PD : MAX_OMEGA_PD;
}

static bool is_left_turn(PathType t)
{
    return t == PATH_ACUTE_LEFT || t == PATH_RIGHT_ANGLE_LEFT || t == PATH_CURVE_LEFT;
}

static bool is_right_turn(PathType t)
{
    return t == PATH_ACUTE_RIGHT || t == PATH_RIGHT_ANGLE_RIGHT || t == PATH_CURVE_RIGHT;
}

static bool is_sharp_turn(PathType t)
{
    return t == PATH_ACUTE_LEFT || t == PATH_ACUTE_RIGHT ||
           t == PATH_RIGHT_ANGLE_LEFT || t == PATH_RIGHT_ANGLE_RIGHT;
}

static float follow_speed(const PathInfo *p, float want)
{
    float v = want;
    if (!p->far_ok && v > CAUTION_SPEED) {
        v = CAUTION_SPEED;
    }
    if (g_slow_frames > 0 && v > CURVE_SPEED) {
        v = CURVE_SPEED;
        g_slow_frames--;
    }
    return v;
}

static void apply_follow(const PathInfo *p)
{
    remember_dir(p);

    switch (p->type) {
    case PATH_STRAIGHT:
    case PATH_CROSS: {
        float om = lock_omega(pd_omega((int)fused_error(p), KP, KD, MAX_OMEGA_PD));
        drive(follow_speed(p, BASE_SPEED), om);
        break;
    }
    case PATH_CURVE_LEFT:
    case PATH_CURVE_RIGHT: {
        float om = lock_omega(pd_omega((int)fused_error(p), KP + 0.08f, KD, MAX_OMEGA_PD + 4.0f));
        drive(follow_speed(p, CURVE_SPEED), om);
        break;
    }
    case PATH_ACUTE_LEFT:
    case PATH_ACUTE_RIGHT: {
        float om = lock_omega(pd_omega((int)fused_error(p), KP_ACUTE, KD_ACUTE, MAX_OMEGA_ACUTE));
        drive(follow_speed(p, ACUTE_SPEED), om);
        break;
    }
    default:
        break;
    }
}

static bool corner_reacquired(const PathInfo *p)
{
    /* 旋转结束后：远端重新出现在中轴附近 */
    if (!p->far_ok) {
        return false;
    }
    int c = s_img_w / 2;
    return abs(p->far_cx - c) < (s_img_w / 6) && abs(p->heading) < (s_img_w / 3);
}

static void apply_corner(const PathInfo *p)
{
    int64_t now = esp_timer_get_time();
    if (now - g_corner_t0 > (int64_t)CORNER_SPIN_TIMEOUT_MS * 1000) {
        ESP_LOGW(TAG, "直角旋转超时，改回寻线");
        g_mode = MODE_SEARCH;
        return;
    }

    if (corner_reacquired(p) && (now - g_corner_t0) > 180000) {
        g_mode = MODE_FOLLOW;
        g_wrong_corner = 0;
        g_last_err = 0.0f;
        apply_follow(p);
        return;
    }

    /* 预瞄方向和正在转的直角相反：说明 90 判反了，立刻改跟线 */
    if (p->far_ok && abs(p->heading) > (s_img_w / 5)) {
        bool head_left = p->heading < 0;
        bool corner_left = (g_corner_dir == PATH_RIGHT_ANGLE_LEFT);
        if (head_left != corner_left) {
            g_wrong_corner++;
            if (g_wrong_corner >= 2) {
                ESP_LOGW(TAG, "直角方向与预瞄相反，改跟线");
                g_mode = MODE_FOLLOW;
                g_wrong_corner = 0;
                g_dir_lock_until = 0;
                PathInfo tmp = *p;
                tmp.type = head_left ? PATH_ACUTE_LEFT : PATH_ACUTE_RIGHT;
                apply_follow(&tmp);
                return;
            }
        } else {
            g_wrong_corner = 0;
        }
    }

    /* 先慢速贴近拐点，再原地旋转。omega<0 左转，>0 右转（与 cam_motion PD 同号） */
    bool left = (g_corner_dir == PATH_RIGHT_ANGLE_LEFT);
    float om = left ? -OMEGA_CORNER : OMEGA_CORNER;

    if ((now - g_corner_t0) < 220000 && p->near_ok) {
        drive(CORNER_APPROACH, om * 0.35f);
    } else {
        drive(0.0f, om);
    }
}

static void apply_search(void)
{
    float om = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
    drive(0.0f, om);
}

static void control_once(void)
{
    PathInfo p = classify_path();

    /* 禁止左右瞬间对打：锁方向，反方向弯道当缓弯跟随 */
    int64_t now_us = esp_timer_get_time();
    if (now_us < g_dir_lock_until) {
        if (g_dir_lock == LAST_DIR_LEFT && is_right_turn(p.type)) {
            p.type = PATH_CURVE_LEFT;
        } else if (g_dir_lock == LAST_DIR_RIGHT && is_left_turn(p.type)) {
            p.type = PATH_CURVE_RIGHT;
        }
    } else if (is_sharp_turn(p.type)) {
        g_dir_lock = is_left_turn(p.type) ? LAST_DIR_LEFT : LAST_DIR_RIGHT;
        g_dir_lock_until = now_us + (int64_t)DIR_LOCK_MS * 1000;
    }

    if (!is_sharp_turn(p.type) && g_hold_frames > 0 && is_sharp_turn(g_hold_turn)) {
        if (!(is_left_turn(p.type) && is_right_turn(g_hold_turn)) &&
            !(is_right_turn(p.type) && is_left_turn(g_hold_turn))) {
            p.type = g_hold_turn;
            g_hold_frames--;
        }
    } else if (is_sharp_turn(p.type)) {
        g_hold_turn = p.type;
        g_hold_frames = HOLD_TURN_FRAMES;
        g_slow_frames = SLOW_HOLD_FRAMES;
    } else {
        g_hold_frames = 0;
        g_hold_turn = PATH_LOST;
    }

    if (p.type == PATH_CURVE_LEFT || p.type == PATH_CURVE_RIGHT ||
        p.type == PATH_RIGHT_ANGLE_LEFT || p.type == PATH_RIGHT_ANGLE_RIGHT) {
        g_slow_frames = SLOW_HOLD_FRAMES;
    }

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG, "%s n=%d f=%d h=%d LR=%d/%d md=%d vy=%.0f om=%.0f D=%.0f A=%.0f B=%.0f %dms",
                 path_name(p.type), p.near_cx, p.far_cx, p.heading,
                 p.left_mass, p.right_mass, (int)g_mode,
                 g_last_vy, g_last_om, g_last_wheels.D, g_last_wheels.A, g_last_wheels.B,
                 g_decode_ms);
        last_log = now;
    }

    if (p.type == PATH_LOST) {
        g_lost_frames++;
        if (g_lost_frames > LOST_STOP_FRAMES && g_mode != MODE_CORNER) {
            stop_motors();
            ESP_LOGE(TAG, "连续丢线，停车保护");
            vTaskDelay(pdMS_TO_TICKS(80));
            return;
        }
        if (g_mode != MODE_CORNER) {
            g_mode = MODE_SEARCH;
        }
    } else {
        g_lost_frames = 0;
    }

    if (g_mode == MODE_CORNER) {
        apply_corner(&p);
        return;
    }

    if (p.type == PATH_RIGHT_ANGLE_LEFT || p.type == PATH_RIGHT_ANGLE_RIGHT) {
        g_corner_hits++;
        if (g_corner_hits < CORNER_CONFIRM_FRAMES) {
            PathInfo tmp = p;
            tmp.type = (p.type == PATH_RIGHT_ANGLE_LEFT) ? PATH_ACUTE_LEFT : PATH_ACUTE_RIGHT;
            g_mode = MODE_FOLLOW;
            apply_follow(&tmp);
            return;
        }
        g_mode = MODE_CORNER;
        g_corner_dir = p.type;
        g_corner_t0 = now;
        g_wrong_corner = 0;
        remember_dir(&p);
        apply_corner(&p);
        return;
    }
    g_corner_hits = 0;

    if (p.type == PATH_LOST || g_mode == MODE_SEARCH) {
        if (p.type != PATH_LOST) {
            g_mode = MODE_FOLLOW;
            apply_follow(&p);
        } else {
            apply_search();
        }
        return;
    }

    g_mode = MODE_FOLLOW;
    apply_follow(&p);
}

/* ==================== UVC ==================== */
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

    if (!xfer_a || !xfer_b || !frame_buf || !s_jpeg || !s_gray) {
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
        /* 只处理最新一帧，丢掉积压，提高控制频率 */
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
            vTaskDelay(pdMS_TO_TICKS(5));
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

    motor_init();
    stop_motors();

    if (!camera_start()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    /* USB 在 core 1，循迹放 core 0，栈要给 JPEG 解码留足 */
    xTaskCreatePinnedToCore(line_task, "line_follow", 12288, NULL, 5, NULL, 0);
}
