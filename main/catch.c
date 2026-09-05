/**
 * 推球入网：红蓝各推一次，先看到哪个推哪个，计满 2 个结束。
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
#include "jpeg_decoder.h"

static const char *TAG = "CAM_PUSH";

/* ==================== 电机（与 cam_line_follow 一致） ==================== */
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

#define BASE_SPEED      46.0f
#define APPROACH_SPEED  40.0f   /* 点动脉冲幅值，不是连续走 */
#define PUSH_SPEED      32.0f   /* 推球：缓慢匀速 */
#define BACKUP_SPEED    36.0f
#define OMEGA_MICRO     12.0f
#define OMEGA_MACRO     18.0f
#define ROT_MAX         22.0f
#define SPIN_PWM        42.0f   /* 点动转的脉冲幅值，要能克服静摩擦 */
#define PIVOT_PWM       42.0f
#define PULSE_ON_MS     120     /* 点动时长 ×4 */
#define PULSE_SPIN_ON_MS 20     /* 搜球/对中：真 20ms 后立刻停，再看下一帧 */
#define PULSE_BACKUP_ON_MS 220  /* 后退每下比靠近走得更远 */
#define PULSE_OFF_MS    80      /* 前进/后退点动间隔；搜球不等这段 */
#define PULSE_BRAKE_SCALE 0.32f /* 反向制动幅值，低于起步静摩擦 */
#define PULSE_BRAKE_MAX_MS 24   /* 制动必须远短于正向，避免反走 */
#define BACKUP_MS       6000
#define APPROACH_STOP_CM 8.0f
#define PUSH_OK_CM      6.0f

/* ==================== 摄像头 ==================== */
#define CAM_WIDTH           480
#define CAM_HEIGHT          320
#define CAM_FPS             30
#define JPEG_DSCALE         2       /* 1/4：彩色识别比 1/8 更稳 */
#define JPEG_XFER_SIZE      (88 * 1024)
#define CAM_FLIP_UD         1
#define CAM_FLIP_LR         1

#define STAGE_TIMEOUT_US    (18LL * 1000 * 1000)
#define LOCK_HOLD_FRAMES    4
#define STOP_HOLD_FRAMES    3
#define ALIGN_HOLD_FRAMES   3
#define PUSH_OK_HOLD        3
#define COINCIDE_PX         8
#define CENTER_DEAD_PX      6
#define CENTER_MACRO_PX     18
#define ALIGN_VERT_PX       6       /* 解码图上球-网 cx 差，视为几乎竖直 */
#define ALIGN_OK_DEG        10.0f   /* 球-网连线相对竖直小于此角度则直行 */
#define RETURN_LINE_MS      1500

#define MAX_CC_BLOBS        12
#define FLOOD_STACK         4096
#define MIN_BALL_AREA       12
#define MAX_BALL_AREA       2800
#define MIN_NET_AREA        8
#define MAX_NET_AREA        6000
#define MIN_CIRCULARITY     0.45f
#define MIN_CIRCULARITY_BLUE 0.32f
#define BALL_LOCK_RADIUS    14      /* 解码图半径阈值（1/4 尺度） */
#define BLACK_LINE_MIN_PCT  18
#define BALL_FAR_CROP_NUM   1       /* 找球时丢掉画面上方 1/8（远方） */
#define BALL_FAR_CROP_DEN   8
#define NEAR_Y_PCT          70      /* cy 超过画面 70% 视为贴到车头 */
#define NEAR_HOLD_FRAMES    4

/* ==================== 类型 ==================== */
typedef struct {
    float D, A, B;
} MotorSpeed;

typedef enum {
    BALL_RED = 0,
    BALL_BLUE
} BallKind;

typedef enum {
    ST_IDLE = 0,
    ST_SEARCH_BALL,
    ST_APPROACH_BALL,
    ST_LOCK_BALL,
    ST_SEARCH_NET,
    ST_ALIGN,
    ST_PUSH,
    ST_BACKUP,
    ST_SEARCH_BLACK,
    ST_RETURN_END,
    ST_DONE,
    ST_FAIL
} PushState;

typedef enum {
    COLOR_RED = 0,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_BLACK
} ColorId;

typedef struct {
    bool found;
    int cx, cy;
    int x0, y0, x1, y1;
    int area;
    int radius;
    float circularity;
    uint8_t mr, mg, mb;
    int mean_v;
} BlobTarget;

typedef struct {
    BlobTarget red;
    BlobTarget blue;
    BlobTarget ball;            /* 当前要推的球 */
    BlobTarget net;
    bool has_black_line;
    int black_cx;
    float align_deg;
} FrameSight;

/* ==================== 全局 ==================== */
static PushState g_state = ST_IDLE;
static BallKind g_ball_kind = BALL_RED;
static int64_t g_stage_t0 = 0;
static int g_lock_frames = 0;
static int64_t g_backup_until = 0;
static int64_t g_pulse_ready_at = 0;
static int g_stop_hold = 0;
static int g_push_ok_hold = 0;
static int g_decode_ms = 0;
static MotorSpeed g_last_wheels = {0, 0, 0};
static float g_last_vy = 0.0f;
static float g_last_om = 0.0f;
static FrameSight g_sight;
static BlobTarget g_locked_ball;
static bool g_done_red = false;
static bool g_done_blue = false;
static float g_last_d_ball = -1.0f;
static float g_last_d_net = -1.0f;

static SemaphoreHandle_t s_frame_mutex;
static SemaphoreHandle_t s_frame_ready;
static SemaphoreHandle_t s_dbg_mutex;
static uint8_t *s_jpeg;
static volatile uint32_t s_jpeg_len;
static uint8_t *s_rgb;          /* RGB888 解码图 */
static uint8_t *s_mask;         /* 当前颜色掩码 / 调试二值 */
static uint8_t *s_visited;
static uint8_t *s_dbg_gray;
static uint8_t *s_dbg_bin;
static int s_img_w = CAM_WIDTH / 4;
static int s_img_h = CAM_HEIGHT / 4;
static char s_dbg_json[4096];
static int s_dbg_json_len;
static volatile bool s_dbg_ready;
static int g_dbg_w, g_dbg_h;
static FrameSight g_dbg_sight;
static PushState g_dbg_state;
static BallKind g_dbg_kind;

/* ==================== 工具 ==================== */
static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int scaled_px(int px480)
{
    int v = px480 * s_img_w / CAM_WIDTH;
    return (v < 1) ? 1 : v;
}

static const char *state_name(PushState s)
{
    switch (s) {
    case ST_IDLE:           return "IDLE";
    case ST_SEARCH_BALL:    return "SEARCH_BALL";
    case ST_APPROACH_BALL:  return "APPROACH_BALL";
    case ST_LOCK_BALL:      return "LOCK_BALL";
    case ST_SEARCH_NET:     return "SEARCH_NET";
    case ST_ALIGN:          return "ALIGN";
    case ST_PUSH:           return "PUSH";
    case ST_BACKUP:         return "BACKUP";
    case ST_SEARCH_BLACK:   return "SEARCH_BLACK";
    case ST_RETURN_END:     return "RETURN_END";
    case ST_DONE:           return "DONE";
    case ST_FAIL:           return "FAIL";
    default:                return "?";
    }
}

static const char *ball_name(BallKind k)
{
    return (k == BALL_RED) ? "RED" : "BLUE";
}

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
    g_last_vy = 0.0f;
    g_last_om = 0.0f;
    g_last_wheels = z;
}

/* 直行差速：与 cam_line_follow 相同 */
static MotorSpeed make_drive(float vy, float omega)
{
    float fwd = 0.0f;
    if (fabsf(vy) > 1.0f) {
        fwd = clampf(vy, -(float)PWM_CAP, (float)PWM_CAP);
    }
    float rot = clampf(omega, -ROT_MAX, ROT_MAX);

    MotorSpeed s;
    s.D = clampf(fwd + rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.A = clampf(fwd - rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.B = clampf(rot * 0.70f, -(float)PWM_CAP, (float)PWM_CAP);
    g_last_vy = vy;
    g_last_om = rot;
    return s;
}

static void drive(float vy, float omega)
{
    MotorSpeed s = make_drive(vy, omega);
    g_last_wheels = s;
    set_all_motors(&s);
}

static int pulse_brake_ms(int on_ms)
{
    /* 约正向时长的 1/5，且不超过上限，保证刹停而不反转 */
    int b = on_ms / 5;
    if (b < 1) {
        b = 1;
    }
    if (b > PULSE_BRAKE_MAX_MS) {
        b = PULSE_BRAKE_MAX_MS;
    }
    if (b >= on_ms) {
        b = (on_ms > 1) ? (on_ms / 2) : 1;
    }
    return b;
}

static MotorSpeed pulse_brake_cmd(const MotorSpeed *cmd)
{
    float s = PULSE_BRAKE_SCALE;
    MotorSpeed b;
    b.D = clampf(-cmd->D * s, -(float)PWM_CAP, (float)PWM_CAP);
    b.A = clampf(-cmd->A * s, -(float)PWM_CAP, (float)PWM_CAP);
    b.B = clampf(-cmd->B * s, -(float)PWM_CAP, (float)PWM_CAP);
    return b;
}

static void wait_pulse_us(int64_t us)
{
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < us) {
        if (us >= 8000) {
            vTaskDelay(0);
        }
    }
}

/* 真按时长通电→制动→停车。不再把电机一直开到下一帧。 */
static void pulse_apply_ms(const MotorSpeed *cmd, int on_ms, int off_ms)
{
    int64_t now = esp_timer_get_time();
    if (now < g_pulse_ready_at) {
        stop_motors();
        return;
    }
    if (on_ms < 1) {
        on_ms = 1;
    }
    if (off_ms < 0) {
        off_ms = 0;
    }

    g_last_wheels = *cmd;
    set_all_motors(cmd);
    wait_pulse_us((int64_t)on_ms * 1000);

    int brake_ms = pulse_brake_ms(on_ms);
    MotorSpeed brk = pulse_brake_cmd(cmd);
    set_all_motors(&brk);
    wait_pulse_us((int64_t)brake_ms * 1000);
    stop_motors();
    g_last_wheels = *cmd;
    g_pulse_ready_at = esp_timer_get_time() + (int64_t)off_ms * 1000;
}

static void pulse_apply(const MotorSpeed *cmd)
{
    pulse_apply_ms(cmd, PULSE_ON_MS, PULSE_OFF_MS);
}

static void pulse_drive(float vy, float omega)
{
    MotorSpeed s = make_drive(vy, omega);
    pulse_apply(&s);
}

static void pulse_drive_ms(float vy, float omega, int on_ms)
{
    MotorSpeed s = make_drive(vy, omega);
    pulse_apply_ms(&s, on_ms, PULSE_OFF_MS);
}

static void spin_in_place(bool left)
{
    float s = left ? -SPIN_PWM : SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    /* 转完立刻停，下一帧再判断，避免连转大半圈 */
    pulse_apply_ms(&m, PULSE_SPIN_ON_MS, 0);
}

/* 绕左轮 D 逆时针：左轮不动，A、B 与原地左转同向 */
static void pulse_pivot_ccw_on_left(void)
{
    float s = clampf(PIVOT_PWM, 0.0f, (float)PWM_CAP);
    MotorSpeed m = { 0.0f, s, -s };
    g_last_vy = 0.0f;
    g_last_om = -s;
    pulse_apply_ms(&m, PULSE_SPIN_ON_MS, 0);
}

/* 绕右轮 A 顺时针：右轮不动 */
static void pulse_pivot_cw_on_right(void)
{
    float s = clampf(PIVOT_PWM, 0.0f, (float)PWM_CAP);
    MotorSpeed m = { s, 0.0f, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    pulse_apply_ms(&m, PULSE_SPIN_ON_MS, 0);
}

static void enter_state(PushState st)
{
    if (g_state != st) {
        ESP_LOGI(TAG, "STATE %s -> %s  ball=%s",
                 state_name(g_state), state_name(st), ball_name(g_ball_kind));
        g_pulse_ready_at = 0;
        g_stop_hold = 0;
        g_push_ok_hold = 0;
        g_lock_frames = 0;
    }
    g_state = st;
    g_stage_t0 = esp_timer_get_time();
}

/* ==================== 图像基础 ==================== */
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

static void rgb_at(int x, int y, uint8_t *r, uint8_t *g, uint8_t *b)
{
    int mx = map_x(x, s_img_w);
    int my = map_y(y, s_img_h);
    const uint8_t *p = s_rgb + ((size_t)my * s_img_w + mx) * 3;
    *r = p[0];
    *g = p[1];
    *b = p[2];
}

/* UVC MJPEG 常省略 DHT；必须用软件 esp_jpeg + 默认 Huffman，不能用 ROM tjpgd */
static bool jpeg_clip_soi_eoi(const uint8_t *jpg, int len, int *off, int *out_len)
{
    int start = -1;
    int end = -1;
    for (int i = 0; i < len - 1; i++) {
        if (jpg[i] == 0xFF && jpg[i + 1] == 0xD8) {
            start = i;
            break;
        }
    }
    if (start < 0) {
        return false;
    }
    for (int i = len - 2; i > start; i--) {
        if (jpg[i] == 0xFF && jpg[i + 1] == 0xD9) {
            end = i + 2;
            break;
        }
    }
    *off = start;
    /* 个别帧缺 EOI：仍尝试解码剩余数据 */
    *out_len = (end > start) ? (end - start) : (len - start);
    return *out_len >= 128;
}

static bool decode_mjpeg_rgb(const uint8_t *jpg, int len)
{
    int off = 0;
    int jlen = 0;
    if (!jpeg_clip_soi_eoi(jpg, len, &off, &jlen)) {
        static int n;
        if ((n++ % 30) == 0) {
            ESP_LOGW(TAG, "JPEG 无 SOI len=%d head=%02X %02X %02X %02X",
                     len,
                     len > 0 ? jpg[0] : 0, len > 1 ? jpg[1] : 0,
                     len > 2 ? jpg[2] : 0, len > 3 ? jpg[3] : 0);
        }
        return false;
    }

    if (!s_rgb) {
        return false;
    }

    esp_jpeg_image_cfg_t cfg = {
        .indata = (uint8_t *)(jpg + off),
        .indata_size = (uint32_t)jlen,
        .outbuf = s_rgb,
        .outbuf_size = (uint32_t)(CAM_WIDTH * CAM_HEIGHT * 3),
        .out_format = JPEG_IMAGE_FORMAT_RGB888,
        .out_scale = JPEG_IMAGE_SCALE_1_4,  /* 对应原 JPEG_DSCALE=2 */
        .flags = {
            .swap_color_bytes = 0,
        },
    };
    esp_jpeg_image_output_t out = {0};

    esp_err_t err = esp_jpeg_decode(&cfg, &out);
    if (err != ESP_OK) {
        static int n;
        if ((n++ % 20) == 0) {
            ESP_LOGW(TAG, "esp_jpeg_decode=%s len=%d soi_off=%d",
                     esp_err_to_name(err), jlen, off);
        }
        return false;
    }
    if (out.width < 16 || out.height < 16 ||
        out.width > CAM_WIDTH || out.height > CAM_HEIGHT) {
        ESP_LOGW(TAG, "解码尺寸异常 %ux%u", out.width, out.height);
        return false;
    }
    s_img_w = (int)out.width;
    s_img_h = (int)out.height;
    return true;
}

/* ==================== HSV + 目标提取 ==================== */
static void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, int *h, int *s, int *v)
{
    int maxc = r;
    if (g > maxc) maxc = g;
    if (b > maxc) maxc = b;
    int minc = r;
    if (g < minc) minc = g;
    if (b < minc) minc = b;
    int delta = maxc - minc;

    *v = maxc;
    if (maxc == 0) {
        *s = 0;
        *h = 0;
        return;
    }
    *s = delta * 255 / maxc;
    if (delta == 0) {
        *h = 0;
        return;
    }

    int hh;
    if (maxc == r) {
        hh = 60 * (g - b) / delta;
    } else if (maxc == g) {
        hh = 120 + 60 * (b - r) / delta;
    } else {
        hh = 240 + 60 * (r - g) / delta;
    }
    if (hh < 0) {
        hh += 360;
    }
    *h = hh / 2;    /* OpenCV 风格 0~179 */
}

/*
 * 颜色策略（像素级 HSV + RGB 主导色，块级再滤一次）：
 * 红球：色相两端 + R 明显大于 G/B，避免木头/橙色干扰。
 * 蓝球：放宽暗度(V≥18,S≥28)，色相约 82–155，且 B 主导——深蓝色球
 *       在相机里往往很暗，旧门槛 V≥40 会整块丢掉；灰/黑没有 B>R、B>G。
 * 荧光绿网：高饱和高亮 + 黄绿到绿的色相 + G 远大于 R/B。
 * 黑杆排除：杆上的绿反光通常又暗又细长；块级丢掉低亮度、瘦高竖条，
 *           并按 area*mean_v 选“又大又亮”的网，而不是最大的暗斑。
 */
static bool pixel_is_color(ColorId id, uint8_t r, uint8_t g, uint8_t b, int h, int s, int v)
{
    switch (id) {
    case COLOR_RED:
        if (!((s >= 55 && v >= 40) && (h <= 14 || h >= 165))) {
            return false;
        }
        return (r >= g + 18) && (r >= b + 18);
    case COLOR_GREEN:
        /* CAM_TUNE 绿网默认：H 35–95 S≥45 V≥55，G 主导 */
        if (!((s >= 45 && v >= 55) && (h >= 35 && h <= 95))) {
            return false;
        }
        return (g >= 40) && (g >= r + 18) && (g >= b + 12);
    case COLOR_BLUE:
        /* CAM_TUNE 蓝球默认：H 85–150 S≥20 V≥12，B 主导 */
        if (!((s >= 20 && v >= 12) && (h >= 85 && h <= 150))) {
            return false;
        }
        return (b >= 30) && (b >= r + 16) && (b >= g + 10);
    case COLOR_BLACK:
        return (v <= 55) && (s <= 90);
    default:
        return false;
    }
}

static void build_mask(ColorId id, int y0)
{
    int n = s_img_w * s_img_h;
    memset(s_mask, 0, (size_t)n);
    memset(s_visited, 0, (size_t)n);
    if (y0 < 0) {
        y0 = 0;
    }
    for (int y = y0; y < s_img_h; y++) {
        if ((y & 15) == 0) {
            vTaskDelay(0);
        }
        for (int x = 0; x < s_img_w; x++) {
            uint8_t r, g, b;
            rgb_at(x, y, &r, &g, &b);
            int hh, ss, vv;
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            if (pixel_is_color(id, r, g, b, hh, ss, vv)) {
                s_mask[y * s_img_w + x] = 1;
            }
        }
    }
}

typedef struct {
    int16_t x, y;
} Pt16;

static bool flood_blob(int sx, int sy, BlobTarget *out)
{
    static Pt16 stack[FLOOD_STACK];
    int sp = 0;
    int w = s_img_w;
    int h = s_img_h;
    size_t idx0 = (size_t)sy * w + sx;
    if (!s_mask[idx0] || s_visited[idx0]) {
        return false;
    }

    int minx = sx, maxx = sx, miny = sy, maxy = sy;
    int64_t sumx = 0, sumy = 0;
    int64_t sumr = 0, sumg = 0, sumb = 0, sumv = 0;
    int area = 0;
    int peri = 0;

    stack[sp++] = (Pt16){ (int16_t)sx, (int16_t)sy };
    s_visited[idx0] = 1;

    while (sp > 0) {
        Pt16 p = stack[--sp];
        area++;
        sumx += p.x;
        sumy += p.y;
        {
            uint8_t r, g, b;
            int hh, ss, vv;
            rgb_at(p.x, p.y, &r, &g, &b);
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            sumr += r;
            sumg += g;
            sumb += b;
            sumv += vv;
        }
        if (p.x < minx) minx = p.x;
        if (p.x > maxx) maxx = p.x;
        if (p.y < miny) miny = p.y;
        if (p.y > maxy) maxy = p.y;

        static const int dx[4] = {1, -1, 0, 0};
        static const int dy[4] = {0, 0, 1, -1};
        for (int i = 0; i < 4; i++) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                peri++;
                continue;
            }
            size_t ni = (size_t)ny * w + nx;
            if (!s_mask[ni]) {
                peri++;
                continue;
            }
            if (s_visited[ni]) {
                continue;
            }
            if (sp >= FLOOD_STACK) {
                continue;
            }
            s_visited[ni] = 1;
            stack[sp++] = (Pt16){ (int16_t)nx, (int16_t)ny };
        }
    }

    if (area < 4) {
        return false;
    }

    float circ = 0.0f;
    if (peri > 0) {
        circ = (4.0f * 3.1415926f * (float)area) / ((float)peri * (float)peri);
    }

    out->found = true;
    out->area = area;
    out->cx = (int)(sumx / area);
    out->cy = (int)(sumy / area);
    out->x0 = minx;
    out->y0 = miny;
    out->x1 = maxx;
    out->y1 = maxy;
    out->radius = (int)(0.5f * sqrtf((float)area / 3.1415926f) + 0.5f);
    out->circularity = circ;
    out->mr = (uint8_t)(sumr / area);
    out->mg = (uint8_t)(sumg / area);
    out->mb = (uint8_t)(sumb / area);
    out->mean_v = (int)(sumv / area);
    return true;
}

static void dilate_mask(int times)
{
    int w = s_img_w;
    int h = s_img_h;
    int np = w * h;
    for (int k = 0; k < times; k++) {
        memcpy(s_visited, s_mask, (size_t)np);
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                size_t i = (size_t)y * w + x;
                if (s_visited[i]) {
                    s_mask[i] = 1;
                    continue;
                }
                int hit = 0;
                if (x > 0 && s_visited[i - 1]) {
                    hit = 1;
                }
                if (x + 1 < w && s_visited[i + 1]) {
                    hit = 1;
                }
                if (y > 0 && s_visited[i - (size_t)w]) {
                    hit = 1;
                }
                if (y + 1 < h && s_visited[i + (size_t)w]) {
                    hit = 1;
                }
                if (hit) {
                    s_mask[i] = 1;
                }
            }
        }
    }
    memset(s_visited, 0, (size_t)np);
}

static bool mask_as_blob(int min_a, BlobTarget *out)
{
    memset(out, 0, sizeof(*out));
    int64_t sumx = 0, sumy = 0, sumr = 0, sumg = 0, sumb = 0, sumv = 0;
    int area = 0;
    int minx = s_img_w, maxx = 0, miny = s_img_h, maxy = 0;
    for (int y = 0; y < s_img_h; y++) {
        for (int x = 0; x < s_img_w; x++) {
            if (!s_mask[y * s_img_w + x]) {
                continue;
            }
            area++;
            sumx += x;
            sumy += y;
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
            uint8_t r, g, b;
            int hh, ss, vv;
            rgb_at(x, y, &r, &g, &b);
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            sumr += r;
            sumg += g;
            sumb += b;
            sumv += vv;
        }
    }
    if (area < min_a) {
        return false;
    }
    out->found = true;
    out->area = area;
    out->cx = (int)(sumx / area);
    out->cy = (int)(sumy / area);
    out->x0 = minx;
    out->y0 = miny;
    out->x1 = maxx;
    out->y1 = maxy;
    out->radius = (int)(0.5f * sqrtf((float)area / 3.1415926f) + 0.5f);
    out->circularity = 0.0f;
    out->mr = (uint8_t)(sumr / area);
    out->mg = (uint8_t)(sumg / area);
    out->mb = (uint8_t)(sumb / area);
    out->mean_v = (int)(sumv / area);
    return true;
}

static int ball_crop_y0(void)
{
    return s_img_h * BALL_FAR_CROP_NUM / BALL_FAR_CROP_DEN;
}

static bool find_best_blob(ColorId id, bool need_round, int min_a, int max_a, BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    int y0 = need_round ? ball_crop_y0() : 0;
    build_mask(id, y0);
    if (!need_round) {
        dilate_mask(2);
    }

    BlobTarget cand;
    BlobTarget largest_any;
    memset(&largest_any, 0, sizeof(largest_any));
    int n_ok = 0;
    int best_score = -1;
    float circ_min = MIN_CIRCULARITY;
    float ar_min = 0.55f;
    if (need_round && id == COLOR_BLUE) {
        circ_min = MIN_CIRCULARITY_BLUE;
        ar_min = 0.45f;
    }
    for (int y = y0; y < s_img_h; y++) {
        for (int x = 0; x < s_img_w; x++) {
            size_t i = (size_t)y * s_img_w + x;
            if (!s_mask[i] || s_visited[i]) {
                continue;
            }
            memset(&cand, 0, sizeof(cand));
            if (!flood_blob(x, y, &cand)) {
                continue;
            }
            if (!largest_any.found || cand.area > largest_any.area) {
                largest_any = cand;
            }
            if (cand.area < min_a || cand.area > max_a) {
                continue;
            }
            if (need_round && cand.circularity < circ_min) {
                continue;
            }
            int bw = cand.x1 - cand.x0 + 1;
            int bh = cand.y1 - cand.y0 + 1;
            float ar = (bw < bh) ? (float)bw / (float)bh : (float)bh / (float)bw;
            if (need_round && ar < ar_min) {
                continue;
            }
            if (need_round && id == COLOR_RED) {
                if (!(cand.mr >= cand.mg + 12 && cand.mr >= cand.mb + 12)) {
                    continue;
                }
            }
            if (need_round && id == COLOR_BLUE) {
                if (!(cand.mb >= cand.mr + 8 && cand.mb >= cand.mg + 5)) {
                    continue;
                }
            }
            if (!need_round) {
                if (ar < 0.22f && bw < (s_img_w / 8 + 1)) {
                    continue;
                }
                if (cand.mean_v < 50) {
                    continue;
                }
                if (bh * 10 >= bw * 20 && bw < (s_img_w * 17 / 100 + 1)) {
                    continue;
                }
                if (!(cand.mg >= cand.mr + 12 && cand.mg >= cand.mb + 8)) {
                    continue;
                }
            }
            if (need_round && id == COLOR_BLUE) {
                if (bh * 10 >= bw * 18 && bw < (s_img_w * 17 / 100 + 1)) {
                    continue;
                }
            }
            int score = need_round ? cand.area : (cand.area * (cand.mean_v + 1));
            n_ok++;
            if (!best->found || score > best_score) {
                *best = cand;
                best_score = score;
            }
            if (n_ok >= MAX_CC_BLOBS) {
                goto done;
            }
        }
    }
done:
    if (!best->found && !need_round) {
        if (largest_any.found && largest_any.area >= 8) {
            int bw = largest_any.x1 - largest_any.x0 + 1;
            int bh = largest_any.y1 - largest_any.y0 + 1;
            bool pole = (bh * 10 >= bw * 20) && (bw < s_img_w / 8 + 1);
            if (!pole) {
                *best = largest_any;
            }
        }
        if (!best->found) {
            mask_as_blob(8, best);
        }
    }
    return best->found;
}

static bool detect_black_line(int *cx_out)
{
    build_mask(COLOR_BLACK, 0);
    int y0 = s_img_h * 55 / 100;
    int y1 = s_img_h - 1;
    int total = 0;
    int black = 0;
    int64_t sumx = 0;
    for (int y = y0; y <= y1; y++) {
        for (int x = 0; x < s_img_w; x++) {
            total++;
            if (s_mask[y * s_img_w + x]) {
                black++;
                sumx += x;
            }
        }
    }
    if (total <= 0) {
        return false;
    }
    int pct = black * 100 / total;
    if (pct < BLACK_LINE_MIN_PCT) {
        return false;
    }
    if (cx_out) {
        *cx_out = (black > 0) ? (int)(sumx / black) : s_img_w / 2;
    }
    return true;
}

/* 车心取画面底部中心（近端） */
static void car_center(int *cx, int *cy)
{
    *cx = s_img_w / 2;
    *cy = s_img_h - 2;
}

/* 球→网连线相对竖直的夹角：正=网在右侧 */
static float align_angle_deg(const BlobTarget *ball, const BlobTarget *net)
{
    float dx = (float)(net->cx - ball->cx);
    float dy = (float)(ball->cy - net->cy); /* 画面向上为正 */
    if (fabsf(dx) < 0.5f && fabsf(dy) < 0.5f) {
        return 0.0f;
    }
    return atan2f(dx, dy) * 180.0f / 3.1415926f;
}

static bool line_almost_vertical(const FrameSight *p)
{
    if (!p || !p->ball.found || !p->net.found) {
        return false;
    }
    int ldx = p->net.cx - p->ball.cx;
    return (fabsf(p->align_deg) <= ALIGN_OK_DEG) || (abs(ldx) <= ALIGN_VERT_PX);
}

static int balls_done(void)
{
    return (g_done_red ? 1 : 0) + (g_done_blue ? 1 : 0);
}

static void pick_search_target(FrameSight *out)
{
    bool take_red = !g_done_red && out->red.found;
    bool take_blue = !g_done_blue && out->blue.found;
    if (take_red && take_blue) {
        if (out->red.cy >= out->blue.cy) {
            g_ball_kind = BALL_RED;
            out->ball = out->red;
        } else {
            g_ball_kind = BALL_BLUE;
            out->ball = out->blue;
        }
        return;
    }
    if (take_red) {
        g_ball_kind = BALL_RED;
        out->ball = out->red;
        return;
    }
    if (take_blue) {
        g_ball_kind = BALL_BLUE;
        out->ball = out->blue;
        return;
    }
    memset(&out->ball, 0, sizeof(out->ball));
}

static void analyze_frame(FrameSight *out)
{
    memset(out, 0, sizeof(*out));
    find_best_blob(COLOR_RED, true, MIN_BALL_AREA, MAX_BALL_AREA, &out->red);
    vTaskDelay(0);
    find_best_blob(COLOR_BLUE, true, MIN_BALL_AREA, MAX_BALL_AREA, &out->blue);
    vTaskDelay(0);
    find_best_blob(COLOR_GREEN, false, MIN_NET_AREA, MAX_NET_AREA, &out->net);
    if (g_state == ST_SEARCH_BLACK || g_state == ST_RETURN_END) {
        out->has_black_line = detect_black_line(&out->black_cx);
    }

    if (g_state == ST_SEARCH_BALL || g_state == ST_IDLE) {
        pick_search_target(out);
    } else if (g_ball_kind == BALL_RED) {
        out->ball = out->red;
    } else {
        out->ball = out->blue;
    }

    if (out->ball.found && out->net.found) {
        out->align_deg = align_angle_deg(&out->ball, &out->net);
    }
}

static bool blob_near_bumper(const BlobTarget *b)
{
    if (!b || !b->found) {
        return false;
    }
    int y_lim = s_img_h * NEAR_Y_PCT / 100;
    return (b->cy >= y_lim) || (b->y1 >= s_img_h * 82 / 100);
}

/* 近处约 20cm 地面映在画面下半，用来把像素换成厘米 */
static float px_per_cm(void)
{
    if (s_img_h < 8) {
        return 2.0f;
    }
    return (float)s_img_h * 0.50f / 20.0f;
}

static float dist_bumper_cm(const BlobTarget *b)
{
    if (!b || !b->found || s_img_h <= 1) {
        return 99.0f;
    }
    int from_bottom = s_img_h - 1 - b->cy;
    if (from_bottom < 0) {
        from_bottom = 0;
    }
    return (float)from_bottom / px_per_cm();
}

static float dist_ball_net_cm(const BlobTarget *b, const BlobTarget *n)
{
    if (!b || !n || !b->found || !n->found) {
        return 99.0f;
    }
    float dx = (float)(b->cx - n->cx);
    float dy = (float)(b->cy - n->cy);
    return sqrtf(dx * dx + dy * dy) / px_per_cm();
}

static int center_dead_px(void)
{
    return scaled_px(CENTER_DEAD_PX * CAM_WIDTH / 60);
}

static int center_macro_px(void)
{
    return scaled_px(CENTER_MACRO_PX * CAM_WIDTH / 60);
}

/* ==================== 运动辅助 ==================== */
static void pulse_center_on_x(int tx, float speed)
{
    int dead = center_dead_px();
    int macro = center_macro_px();
    int err = tx - s_img_w / 2;
    /* 贴球不再前进时：用搜球同款 2ms 旋转点动对中，避免 30ms 差速猛拧 */
    if (fabsf(speed) <= 1.0f) {
        if (abs(err) <= dead) {
            stop_motors();
            return;
        }
        spin_in_place(err < 0);
        return;
    }
    if (abs(err) <= dead) {
        pulse_drive(speed, 0.0f);
        return;
    }
    float om = (abs(err) <= macro) ? OMEGA_MICRO : OMEGA_MACRO;
    if (err < 0) {
        om = -om;
    }
    pulse_drive(speed, om);
}

static bool stage_timeout(void)
{
    return (esp_timer_get_time() - g_stage_t0) > STAGE_TIMEOUT_US;
}

/* ==================== 调试可视化（对齐 cam_line_follow） ==================== */
static void debug_copy_images(void)
{
    int w = s_img_w;
    int h = s_img_h;
    for (int y = 0; y < h; y++) {
        uint8_t *dg = s_dbg_gray + y * w;
        uint8_t *db = s_dbg_bin + y * w;
        for (int x = 0; x < w; x++) {
            uint8_t r, g, b;
            rgb_at(x, y, &r, &g, &b);
            dg[x] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
            /* 右图：掩码叠加，红/绿/蓝目标高亮 */
            db[x] = 40;
            if (s_mask[y * w + x]) {
                db[x] = 0;
            }
        }
    }
}

static void debug_update(const FrameSight *p)
{
    static int dbg_n;
    if ((++dbg_n & 1) != 0) {
        return;
    }
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin) {
        return;
    }
    if (xSemaphoreTake(s_dbg_mutex, 0) != pdTRUE) {
        return;
    }
    /* 右图掩码跟当前阶段相关：搜/接近球→球色；搜网/对齐/推→绿；回终点→黑 */
    ColorId mask_c = COLOR_RED;
    if (g_state == ST_SEARCH_NET || g_state == ST_ALIGN || g_state == ST_PUSH) {
        mask_c = COLOR_GREEN;
    } else if (g_state == ST_SEARCH_BLACK || g_state == ST_RETURN_END || g_state == ST_BACKUP) {
        mask_c = COLOR_BLACK;
    } else {
        mask_c = (g_ball_kind == BALL_RED) ? COLOR_RED : COLOR_BLUE;
    }
    build_mask(mask_c, (mask_c == COLOR_RED || mask_c == COLOR_BLUE) ? ball_crop_y0() : 0);

    g_dbg_sight = *p;
    g_dbg_state = g_state;
    g_dbg_kind = g_ball_kind;
    g_dbg_w = s_img_w;
    g_dbg_h = s_img_h;
    debug_copy_images();

    int line_dx = (p->ball.found && p->net.found) ? (p->net.cx - p->ball.cx) : 0;
    float d_ball = p->ball.found ? dist_bumper_cm(&p->ball) : -1.0f;
    float d_net = (p->ball.found && p->net.found) ? dist_ball_net_cm(&p->ball, &p->net) : -1.0f;
    g_last_d_ball = d_ball;
    g_last_d_net = d_net;

    int ccx, ccy;
    car_center(&ccx, &ccy);
    int n = snprintf(s_dbg_json, sizeof(s_dbg_json),
                     "{\"st\":\"%s\",\"ball\":\"%s\",\"ms\":%d,\"vy\":%.0f,\"om\":%.0f,"
                     "\"ang\":%.1f,\"ccx\":%d,\"ccy\":%d,\"crop\":%d,"
                     "\"dcm\":%.1f,\"ndcm\":%.1f,\"ldx\":%d,"
                     "\"done\":{\"r\":%d,\"u\":%d,\"n\":%d},"
                     "\"b\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"r\":%d,\"a\":%d,\"c\":%.2f,"
                     "\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"red\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"blu\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"n\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"a\":%d,"
                     "\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"blk\":{\"ok\":%d,\"cx\":%d},"
                     "\"lock\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d}}",
                     state_name(g_state), ball_name(g_ball_kind), g_decode_ms, g_last_vy, g_last_om,
                     p->align_deg, ccx, ccy, ball_crop_y0(),
                     d_ball, d_net, line_dx,
                     g_done_red ? 1 : 0, g_done_blue ? 1 : 0, balls_done(),
                     p->ball.found ? 1 : 0, p->ball.cx, p->ball.cy, p->ball.radius, p->ball.area,
                     p->ball.circularity, p->ball.x0, p->ball.y0, p->ball.x1, p->ball.y1,
                     p->red.found ? 1 : 0, p->red.cx, p->red.cy, p->red.x0, p->red.y0, p->red.x1, p->red.y1,
                     p->blue.found ? 1 : 0, p->blue.cx, p->blue.cy, p->blue.x0, p->blue.y0, p->blue.x1, p->blue.y1,
                     p->net.found ? 1 : 0, p->net.cx, p->net.cy, p->net.area,
                     p->net.x0, p->net.y0, p->net.x1, p->net.y1,
                     p->has_black_line ? 1 : 0, p->black_cx,
                     g_locked_ball.found ? 1 : 0, g_locked_ball.cx, g_locked_ball.cy,
                     g_locked_ball.x0, g_locked_ball.y0, g_locked_ball.x1, g_locked_ball.y1);
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
    "<!DOCTYPE html><html><head><meta charset=utf-8><title>推球入网</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#ddd;margin:16px;max-width:1100px}"
    ".row{display:flex;flex-wrap:wrap;gap:16px}"
    "figure{margin:0}"
    "canvas{background:#000;image-rendering:pixelated;width:min(46vw,480px);border:1px solid #444}"
    "figcaption{color:#aaa;margin:6px 0 0}"
    "pre{background:#1a1a1a;padding:10px;line-height:1.45;white-space:pre-wrap}"
    "ol{line-height:1.55;color:#ccc} li{margin:6px 0}</style></head><body>"
    "<h2>推球入网调试</h2>"
    "<div class=row>"
    "<figure><canvas id=a width=60 height=40></canvas>"
    "<figcaption>左：原图。红框=球　绿框=网　青线以上=找球时丢掉的远方 1/8</figcaption></figure>"
    "<figure><canvas id=b width=60 height=40></canvas>"
    "<figcaption>右：最近一次颜色掩码。黑=命中</figcaption></figure>"
    "</div>"
    "<pre id=t>连接中...</pre>"
    "<h3>流程</h3>"
    "<ol>"
    "<li>点动慢转，先看到哪颗未入网的球就推哪颗。</li>"
    "<li>点动靠近到约 8cm，把球停在画面下方中间。</li>"
    "<li>球-网连线相对竖直小于 10° 则直接匀速直行；否则左偏绕左轮、右偏绕右轮点动对齐。</li>"
    "<li>匀速前推；球与网距离小于约 6cm 算入网，然后点动后退 6 秒，再找下一颗。</li>"
    "</ol>"
    "<script>"
    "function paint(cv,pix,w,h){cv.width=w;cv.height=h;const ctx=cv.getContext('2d');"
    "const im=ctx.createImageData(w,h);"
    "for(let i=0;i<w*h;i++){const v=pix[i]||0;im.data[i*4]=v;im.data[i*4+1]=v;im.data[i*4+2]=v;im.data[i*4+3]=255;}"
    "ctx.putImageData(im,0,0);return ctx;}"
    "function box(ctx,o,c){if(!o||!o.ok)return;ctx.strokeStyle=c;ctx.lineWidth=2;"
    "ctx.strokeRect(o.x0+0.5,o.y0+0.5,Math.max(2,o.x1-o.x0),Math.max(2,o.y1-o.y0));"
    "ctx.fillStyle=c;ctx.beginPath();ctx.arc(o.cx,o.cy,2.5,0,6.28);ctx.fill();}"
    "let busy=false;"
    "async function tick(){"
    "if(busy)return;busy=true;"
    "try{"
    "const r=await fetch('/snap');"
    "if(!r.ok)throw new Error('HTTP '+r.status);"
    "const buf=new Uint8Array(await r.arrayBuffer());"
    "const jl=buf[0]|buf[1]<<8;"
    "const inf=JSON.parse(new TextDecoder().decode(buf.subarray(2,2+jl)));"
    "const o=2+jl,w=buf[o]|buf[o+1]<<8,h=buf[o+2]|buf[o+3]<<8,n=w*h;"
    "const orig=buf.subarray(o+4,o+4+n),bin=buf.subarray(o+4+n,o+4+2*n);"
    "const c1=paint(document.getElementById('a'),orig,w,h);"
    "const c2=paint(document.getElementById('b'),bin,w,h);"
    "if(inf.crop>0){c1.strokeStyle='#0ff';c1.beginPath();c1.moveTo(0,inf.crop+0.5);c1.lineTo(w,inf.crop+0.5);c1.stroke();}"
    "box(c1,inf.red,'#f44');box(c1,inf.blu,'#4af');box(c1,inf.n,'#0f6');box(c1,inf.lock,'#fa0');"
    "box(c2,inf.b,'#f44');box(c2,inf.n,'#0f6');"
    "if(inf.b&&inf.b.ok&&inf.n&&inf.n.ok){c1.strokeStyle='#fc0';c1.lineWidth=2;c1.beginPath();"
    "c1.moveTo(inf.b.cx,inf.b.cy);c1.lineTo(inf.n.cx,inf.n.cy);c1.stroke();}"
    "c1.strokeStyle='#ff0';c1.lineWidth=1;c1.beginPath();"
    "c1.moveTo(inf.ccx,inf.ccy);"
    "if(inf.b&&inf.b.ok){c1.lineTo(inf.b.cx,inf.b.cy);}"
    "c1.stroke();"
    "c1.fillStyle='#0ff';c1.beginPath();c1.arc(inf.ccx,inf.ccy,3,0,6.28);c1.fill();"
    "document.getElementById('t').textContent="
    "'状态：'+inf.st+'    目标：'+inf.ball+'    已推 '+(inf.done?inf.done.n:0)+'/2'+"
    "'  红='+(inf.done&&inf.done.r?'已入':'未')+' 蓝='+(inf.done&&inf.done.u?'已入':'未')+"
    "'\\n当前球 cx,cy,r,a = '+ (inf.b&&inf.b.ok? [inf.b.cx,inf.b.cy,inf.b.r,inf.b.a].join(','):'-') +"
    "'\\n网 cx,cy,a = '+ (inf.n? [inf.n.cx,inf.n.cy,inf.n.a].join(','):'-') +"
    "'\\n球距车头 dcm='+(inf.dcm!=null?inf.dcm:'-')+'cm  球-网 ndcm='+(inf.ndcm!=null?inf.ndcm:'-')+'cm  连线ldx='+inf.ldx+"
    "'\\n对齐角 ang='+inf.ang+'°    黑线='+(inf.blk&&inf.blk.ok?'Y@'+inf.blk.cx:'N') +"
    "'\\nvy='+inf.vy+' om='+inf.om+'  解码 '+inf.ms+'ms';"
    "}catch(e){document.getElementById('t').textContent='等待画面 '+e;}"
    "busy=false;}"
    "setInterval(tick,400);tick();"
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
    esp_err_t er = httpd_resp_send(req, (const char *)pkt, (ssize_t)total);
    free(pkt);
    return er;
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
    memcpy(wifi_config.ap.ssid, "CAM_PUSH", 8);
    wifi_config.ap.ssid_len = 8;
    wifi_config.ap.channel = 1;
    memcpy(wifi_config.ap.password, "12345678", 8);
    wifi_config.ap.max_connection = 4;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;
    config.stack_size = 8192;
    config.core_id = tskNO_AFFINITY;
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
    ESP_LOGI(TAG, "调试页: 连 WiFi CAM_PUSH 密码 12345678，浏览器打开 http://192.168.4.1/");
}

/* ==================== 状态机 ==================== */
static void mark_ball_done(void)
{
    if (g_ball_kind == BALL_RED) {
        g_done_red = true;
        ESP_LOGI(TAG, "红球已入网  计数 %d/2", balls_done());
    } else {
        g_done_blue = true;
        ESP_LOGI(TAG, "蓝球已入网  计数 %d/2", balls_done());
    }
    memset(&g_locked_ball, 0, sizeof(g_locked_ball));
}

static void on_push_success(void)
{
    mark_ball_done();
    stop_motors();
    enter_state(ST_BACKUP);
    g_backup_until = esp_timer_get_time() + (int64_t)BACKUP_MS * 1000;
}

static void after_backup(void)
{
    stop_motors();
    if (balls_done() >= 2) {
        ESP_LOGI(TAG, "红蓝都已推入，任务结束");
        enter_state(ST_DONE);
        return;
    }
    ESP_LOGI(TAG, "后退结束，点动找下一颗球");
    enter_state(ST_SEARCH_BALL);
    spin_in_place(true);
}

static void control_once(void)
{
    analyze_frame(&g_sight);
    FrameSight *p = &g_sight;
    g_last_d_ball = p->ball.found ? dist_bumper_cm(&p->ball) : -1.0f;
    g_last_d_net = (p->ball.found && p->net.found) ? dist_ball_net_cm(&p->ball, &p->net) : -1.0f;

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG,
                 "%s tgt=%s done=%d/2 R%s(%d,%d) U%s(%d,%d) N(%d,%d) "
                 "d=%.1f nd=%.1f vy=%.0f om=%.0f %dms",
                 state_name(g_state), ball_name(g_ball_kind), balls_done(),
                 g_done_red ? "OK" : "",
                 p->red.found ? p->red.cx : -1,
                 p->red.found ? p->red.cy : -1,
                 g_done_blue ? "OK" : "",
                 p->blue.found ? p->blue.cx : -1,
                 p->blue.found ? p->blue.cy : -1,
                 p->net.found ? p->net.cx : -1,
                 p->net.found ? p->net.cy : -1,
                 g_last_d_ball, g_last_d_net, g_last_vy, g_last_om, g_decode_ms);
        last_log = now;
    }

    if (g_state != ST_DONE && g_state != ST_FAIL && g_state != ST_IDLE &&
        g_state != ST_BACKUP && g_state != ST_PUSH &&
        g_state != ST_SEARCH_BALL &&
        stage_timeout()) {
        ESP_LOGW(TAG, "阶段超时，回到搜球 @ %s", state_name(g_state));
        stop_motors();
        enter_state(ST_SEARCH_BALL);
        debug_update(p);
        return;
    }

    switch (g_state) {
    case ST_IDLE:
        stop_motors();
        enter_state(ST_SEARCH_BALL);
        break;

    case ST_SEARCH_BALL:
        if (p->ball.found) {
            stop_motors();
            enter_state(ST_APPROACH_BALL);
        } else {
            spin_in_place(true);
        }
        break;

    case ST_APPROACH_BALL: {
        if (!p->ball.found) {
            enter_state(ST_SEARCH_BALL);
            spin_in_place(true);
            break;
        }
        float d = dist_bumper_cm(&p->ball);
        int err = p->ball.cx - s_img_w / 2;
        bool x_ok = abs(err) <= center_dead_px() + 2;
        bool low_ok = (p->ball.cy >= s_img_h * 55 / 100) || blob_near_bumper(&p->ball);
        bool at_range = ((d <= APPROACH_STOP_CM + 1.2f) && low_ok) || blob_near_bumper(&p->ball);
        if (at_range && x_ok) {
            stop_motors();
            g_stop_hold++;
            if (g_stop_hold >= STOP_HOLD_FRAMES) {
                ESP_LOGI(TAG, "停在球前 d=%.1fcm cx=%d cy=%d，开始对齐",
                         d, p->ball.cx, p->ball.cy);
                g_locked_ball = p->ball;
                if (line_almost_vertical(p)) {
                    ESP_LOGI(TAG, "连线 %.1f° 已小于10°，直接直行推球", p->align_deg);
                    enter_state(ST_PUSH);
                    drive(PUSH_SPEED, 0.0f);
                } else {
                    enter_state(ST_ALIGN);
                }
            }
        } else {
            g_stop_hold = 0;
            float fwd = at_range ? 0.0f : APPROACH_SPEED;
            pulse_center_on_x(p->ball.cx, fwd);
        }
        break;
    }

    case ST_LOCK_BALL:
    case ST_SEARCH_NET:
        stop_motors();
        enter_state(ST_ALIGN);
        break;

    case ST_ALIGN: {
        if (!p->ball.found) {
            enter_state(ST_SEARCH_BALL);
            spin_in_place(true);
            break;
        }
        if (!p->net.found) {
            spin_in_place(true);
            break;
        }
        int ldx = p->net.cx - p->ball.cx;
        if (line_almost_vertical(p)) {
            stop_motors();
            g_lock_frames++;
            if (g_lock_frames >= ALIGN_HOLD_FRAMES) {
                ESP_LOGI(TAG, "连线 %.1f° ldx=%d，开始匀速直行", p->align_deg, ldx);
                enter_state(ST_PUSH);
                drive(PUSH_SPEED, 0.0f);
            }
        } else {
            g_lock_frames = 0;
            if (p->align_deg < 0.0f || ldx < 0) {
                /* 连线往左偏：绕左轮逆时针 */
                pulse_pivot_ccw_on_left();
            } else {
                pulse_pivot_cw_on_right();
            }
        }
        break;
    }

    case ST_PUSH: {
        if (p->ball.found && p->net.found) {
            float nd = dist_ball_net_cm(&p->ball, &p->net);
            int dx = p->ball.cx - p->net.cx;
            int dy = p->ball.cy - p->net.cy;
            int overlap = p->ball.radius + 3;
            bool close_ok = (nd <= PUSH_OK_CM) ||
                            (dx * dx + dy * dy <= overlap * overlap);
            if (close_ok) {
                g_push_ok_hold++;
                stop_motors();
                if (g_push_ok_hold >= PUSH_OK_HOLD) {
                    ESP_LOGI(TAG, "推球成功 nd=%.1fcm dx=%d dy=%d", nd, dx, dy);
                    on_push_success();
                }
                break;
            }
            g_push_ok_hold = 0;
            drive(PUSH_SPEED, 0.0f);
        } else {
            /* 球可能已被车头挡住：保持匀速直行 */
            drive(PUSH_SPEED, 0.0f);
        }
        break;
    }

    case ST_BACKUP:
        if (esp_timer_get_time() >= g_backup_until) {
            after_backup();
        } else {
            pulse_drive_ms(-BACKUP_SPEED, 0.0f, PULSE_BACKUP_ON_MS);
        }
        break;

    case ST_SEARCH_BLACK:
    case ST_RETURN_END:
        enter_state(ST_SEARCH_BALL);
        spin_in_place(true);
        break;

    case ST_DONE:
        stop_motors();
        break;

    case ST_FAIL:
        stop_motors();
        ESP_LOGW(TAG, "从 FAIL 恢复，重新搜球");
        enter_state(ST_SEARCH_BALL);
        spin_in_place(true);
        break;

    default:
        enter_state(ST_FAIL);
        break;
    }

    debug_update(p);
}

/* ==================== 摄像头 / 主任务 ==================== */
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
    s_rgb = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT * 3);
    s_mask = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_visited = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);

    if (!xfer_a || !xfer_b || !frame_buf || !s_jpeg || !s_rgb || !s_mask || !s_visited) {
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

static void push_task(void *arg)
{
    (void)arg;
    uint8_t *local_jpg = (uint8_t *)psram_alloc(JPEG_XFER_SIZE);
    if (!local_jpg) {
        ESP_LOGE(TAG, "任务 JPEG 缓冲分配失败");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "推球入网任务启动");
    stop_motors();
    vTaskDelay(pdMS_TO_TICKS(500));
    enter_state(ST_SEARCH_BALL);
    spin_in_place(true);

    while (1) {
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(200)) != pdTRUE) {
            static int miss;
            miss++;
            if ((miss % 10) == 1) {
                ESP_LOGW(TAG, "没有摄像头画面 %d，仍原地搜球。热点 CAM_PUSH", miss);
            }
            if (g_state == ST_SEARCH_BALL || g_state == ST_ALIGN ||
                g_state == ST_IDLE) {
                if (g_state == ST_IDLE) {
                    enter_state(ST_SEARCH_BALL);
                }
                spin_in_place(true);
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
        if (!decode_mjpeg_rgb(local_jpg, (int)len)) {
            ESP_LOGW(TAG, "JPEG 解码失败 len=%u", (unsigned)len);
            continue;
        }
        g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        control_once();
        vTaskDelay(pdMS_TO_TICKS(5));
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
    vTaskDelay(pdMS_TO_TICKS(800));

    if (!camera_start()) {
        ESP_LOGE(TAG, "摄像头初始化失败。热点 CAM_PUSH 仍可用：http://192.168.4.1/");
        while (1) {
            spin_in_place(true);
            vTaskDelay(pdMS_TO_TICKS(700));
            stop_motors();
            vTaskDelay(pdMS_TO_TICKS(700));
        }
    }

    xTaskCreatePinnedToCore(push_task, "push_ball", 16384, NULL, 4, NULL, 0);
}
  