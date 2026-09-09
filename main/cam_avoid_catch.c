/**
 * 摄像头循迹 + 超声波避障（cam_line_avoid_ui）后等待 3 秒，再找球推球（catch.c）。
 *
 * 运动逻辑对齐 cam_line_avoid_ui.c；推球对齐当前 catch.c。
 *
 * 引脚冲突处理：
 *   ESP32-S3 USB-OTG（UVC 摄像头）固定占用 GPIO19 / GPIO20。
 *   main_final.c 右前霍尔原来也在 19/20，这里改接到空闲的 GPIO4 / GPIO5
 *   （原红外循迹脚，本固件不再用红外）。
 *   右前轮霍尔 A/B 请从 19/20 改焊到 4/5，否则 WheelA 转速无效。
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
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"

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

#include "esp_rom_sys.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

#include "usb_stream.h"
#include "tjpgd.h"
#include "jpeg_decoder.h"
#include "esp_task_wdt.h"

static const char *TAG = "AVOID_CATCH";

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

#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000

/* 霍尔：D/B 与 main_final 相同；A 避开 USB 的 19/20 */
#define HALL_D_A_PIN    GPIO_NUM_10
#define HALL_D_B_PIN    GPIO_NUM_11
#define HALL_A_A_PIN    GPIO_NUM_4
#define HALL_A_B_PIN    GPIO_NUM_5
#define HALL_B_A_PIN    GPIO_NUM_42
#define HALL_B_B_PIN    GPIO_NUM_41
#define ENCODER_PPR     512
#define SAMPLE_TIME_MS  100
#define WHEEL_D         0
#define WHEEL_A         1
#define WHEEL_B         2

/* ST7789 128x160，与 main_final 相同 */
#define PIN_NUM_CS      GPIO_NUM_2
#define PIN_NUM_SCK     GPIO_NUM_1
#define PIN_NUM_SDI     GPIO_NUM_38
#define PIN_NUM_DC      GPIO_NUM_39
#define PIN_NUM_RST     GPIO_NUM_40
#define LCD_H_RES       128
#define LCD_V_RES       160

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f
#define STRAFE_SCALE_D  0.87f
#define STRAFE_SCALE_A  0.87f
#define STRAFE_SCALE_B  1.88f     /* 后轮再快一点，抵消左后斜 */

/* ==================== 摄像头 ==================== */
#define CAM_WIDTH           480
#define CAM_HEIGHT          320
#define CAM_FPS             30
#define JPEG_DSCALE         3       /* 1/8，把解码从 ~100ms 打到几十 ms */
#define JPEG_XFER_SIZE      (88 * 1024)
#define CAM_FLIP_UD         1
#define CAM_FLIP_LR         1

/* 检测窗口为上一版的 5/4：底边仍贴车头，5 行=4 层 */
#define SCAN_Y_NEAR         320
#define SCAN_Y_FAR          253
#define SCAN_ROWS           5
#define ROI_X0_480          190
#define ROI_X1_480          290

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

/* 循迹慢而匀速；36 左右才能克服静摩擦 */
#define BASE_SPEED          36.0f
#define OMEGA_MICRO         5.0f
#define OMEGA_MACRO         10.0f
#define ROT_MAX             24.0f
#define SPIN_PWM            31.0f
#define DEAD_PX_480         6
#define MACRO_PX_480        16
#define LOST_STOP_FRAMES    180

#define STRAFE_SPEED        48.0f
#define STRAFE_BACK_SPEED   38.0f   /* 与避障横移同一套轮速比例 */
#define AVOID_TRIGGER_CM    10.0f
#define AVOID_CLEAR_CM      13.0f
#define STRAFE_FORCE_MS     800
#define STRAFE_ALIGN_MS     700
#define AVOID_FWD_MS        1485    /* 1350 再多 1/10 */
#define T_WHITE_FRAMES      3
#define CATCH_WAIT_MS       3000

typedef enum {
    MISSION_LINE = 0,
    MISSION_WAIT,
    MISSION_CATCH,
    MISSION_DONE
} Mission;

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
    PHASE_FOLLOW = 0,
    PHASE_ALIGN,
    PHASE_STRAFE_FORCE,
    PHASE_STRAFE_WAIT,
    PHASE_FWD,
    PHASE_STRAFE_BACK,
    PHASE_STOP
} Phase;

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
    bool t_bar;      /* 近端横着全黑且左右都有：终止 T */
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
static Phase g_phase = PHASE_FOLLOW;
static int g_lost_frames = 0;
static int g_decode_ms = 0;
static MotorSpeed g_last_wheels = {0, 0, 0};
static float g_last_vy = 0.0f;
static float g_last_om = 0.0f;
static float g_last_dist = -1.0f;
static float g_current_distance = 0.0f;
static int64_t g_phase_t0 = 0;
static int g_hit_cm = 0;
static int g_t_white = 0;
static bool g_t_seen = false;
static bool g_has_avoided = false;
static volatile Mission g_mission = MISSION_LINE;
static int64_t g_wait_t0 = 0;
static uint8_t *s_rgb;
static uint8_t *s_mask;
static uint8_t *s_visited;
static uint8_t *s_jpeg_work;
#define JPEG_RGB_WORK_SZ    (16 * 1024)


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

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel;
} EncoderHandle_t;

typedef struct {
    float rpm_D;
    float rpm_A;
    float rpm_B;
} AllWheelRPM;

static EncoderHandle_t g_encoders[3];
static AllWheelRPM g_current_rpm;

static lv_disp_draw_buf_t disp_buf;
static lv_color_t s_lv_buf1[LCD_H_RES * 20];
static lv_color_t s_lv_buf2[LCD_H_RES * 20];
static lv_obj_t *lbl_wheel_a;
static lv_obj_t *lbl_wheel_b;
static lv_obj_t *lbl_wheel_d;
static lv_obj_t *lbl_dist;
static lv_obj_t *lbl_phase;

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

static void ultrasonic_init(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig);
    gpio_config_t echo = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&echo);
    gpio_set_level(TRIG_PIN, 0);
}

static float ultrasonic_cm(void)
{
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - t0 > TIMEOUT_US) {
            return -1.0f;
        }
    }
    int64_t t1 = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - t1 > TIMEOUT_US) {
            return -2.0f;
        }
    }
    float d = (float)(esp_timer_get_time() - t1) * 0.0343f / 2.0f;
    if (d > 0.0f) {
        g_last_dist = d;
        g_current_distance = d;
    }
    return d;
}

static void init_single_encoder(gpio_num_t pin_a, gpio_num_t pin_b, EncoderHandle_t *handle)
{
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000,
        .flags.accum_count = true,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &handle->unit));

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = pin_a,
        .level_gpio_num = pin_b,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(handle->unit, &chan_config, &handle->channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(handle->channel,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(handle->channel,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_enable(handle->unit));
    ESP_ERROR_CHECK(pcnt_unit_start(handle->unit));
}

static void encoder_init(void)
{
    init_single_encoder(HALL_D_A_PIN, HALL_D_B_PIN, &g_encoders[WHEEL_D]);
    init_single_encoder(HALL_A_A_PIN, HALL_A_B_PIN, &g_encoders[WHEEL_A]);
    init_single_encoder(HALL_B_A_PIN, HALL_B_B_PIN, &g_encoders[WHEEL_B]);
}

static AllWheelRPM get_all_wheel_rpm(void)
{
    AllWheelRPM result = {0};
    static int64_t last_time = 0;
    static int32_t last_count_D = 0, last_count_A = 0, last_count_B = 0;
    int64_t now = esp_timer_get_time();

    if (last_time == 0) {
        pcnt_unit_get_count(g_encoders[WHEEL_D].unit, (int *)&last_count_D);
        pcnt_unit_get_count(g_encoders[WHEEL_A].unit, (int *)&last_count_A);
        pcnt_unit_get_count(g_encoders[WHEEL_B].unit, (int *)&last_count_B);
        last_time = now;
        return result;
    }

    int64_t elapsed_us = now - last_time;
    if (elapsed_us < (int64_t)SAMPLE_TIME_MS * 1000) {
        return g_current_rpm;
    }

    int count_D = 0, count_A = 0, count_B = 0;
    pcnt_unit_get_count(g_encoders[WHEEL_D].unit, &count_D);
    pcnt_unit_get_count(g_encoders[WHEEL_A].unit, &count_A);
    pcnt_unit_get_count(g_encoders[WHEEL_B].unit, &count_B);

    int32_t delta_D = count_D - last_count_D;
    int32_t delta_A = count_A - last_count_A;
    int32_t delta_B = count_B - last_count_B;
    last_count_D = count_D;
    last_count_A = count_A;
    last_count_B = count_B;

    float minutes = (float)elapsed_us / 60000000.0f;
    if (minutes > 0.0f) {
        result.rpm_D = ((float)delta_D / (float)ENCODER_PPR) / minutes;
        result.rpm_A = ((float)delta_A / (float)ENCODER_PPR) / minutes;
        result.rpm_B = ((float)delta_B / (float)ENCODER_PPR) / minutes;
    }
    last_time = now;
    g_current_rpm = result;
    return result;
}

static void increase_lvgl_tick(void *arg)
{
    (void)arg;
    lv_tick_inc(2);
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static const char *phase_name_ui(Phase p)
{
    switch (p) {
    case PHASE_FOLLOW:       return "FOLLOW";
    case PHASE_ALIGN:        return "ALIGN";
    case PHASE_STRAFE_FORCE: return "STRAFE";
    case PHASE_STRAFE_WAIT:  return "WAIT";
    case PHASE_FWD:          return "FWD";
    case PHASE_STRAFE_BACK:  return "BACK";
    case PHASE_STOP:         return "STOP";
    default:                 return "?";
    }
}

static void lcd_init_and_ui(void)
{
    ESP_LOGI(TAG, "初始化 LCD SPI");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_SCK,
        .mosi_io_num = PIN_NUM_SDI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_DC,
        .cs_gpio_num = PIN_NUM_CS,
        .pclk_hz = 20 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &io_handle));

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_NUM_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

    lv_init();
    lv_disp_draw_buf_init(&disp_buf, s_lv_buf1, s_lv_buf2, LCD_H_RES * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);
    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 120, 150);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 2, 0);

    lbl_wheel_d = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_d, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_d, "D: 0.0");

    lbl_wheel_a = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_a, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_a, "A: 0.0");

    lbl_wheel_b = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_b, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_b, "B: 0.0");

    lbl_dist = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_dist, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_dist, "US: -- cm");

    lbl_phase = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_phase, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_phase, "PH: FOLLOW");
}

static void gui_task(void *pv)
{
    (void)pv;
    TickType_t last_text = xTaskGetTickCount();
    char buf[40];

    while (1) {
        lv_timer_handler();
        if (xTaskGetTickCount() - last_text >= pdMS_TO_TICKS(200)) {
            last_text = xTaskGetTickCount();
            get_all_wheel_rpm();
            if (lbl_wheel_a && lbl_wheel_b && lbl_wheel_d && lbl_dist && lbl_phase) {
                snprintf(buf, sizeof(buf), "D: %d.%d",
                         (int)g_current_rpm.rpm_D, (int)fabsf(g_current_rpm.rpm_D * 10.0f) % 10);
                lv_label_set_text(lbl_wheel_d, buf);
                snprintf(buf, sizeof(buf), "A: %d.%d",
                         (int)g_current_rpm.rpm_A, (int)fabsf(g_current_rpm.rpm_A * 10.0f) % 10);
                lv_label_set_text(lbl_wheel_a, buf);
                snprintf(buf, sizeof(buf), "B: %d.%d",
                         (int)g_current_rpm.rpm_B, (int)fabsf(g_current_rpm.rpm_B * 10.0f) % 10);
                lv_label_set_text(lbl_wheel_b, buf);
                if (g_current_distance > 0.0f) {
                    snprintf(buf, sizeof(buf), "US: %d.%dcm",
                             (int)g_current_distance, (int)fabsf(g_current_distance * 10.0f) % 10);
                } else {
                    snprintf(buf, sizeof(buf), "US: -- cm");
                }
                lv_label_set_text(lbl_dist, buf);
                snprintf(buf, sizeof(buf), "PH: %s", phase_name_ui(g_phase));
                lv_label_set_text(lbl_phase, buf);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static MotorSpeed inverse_kinematics(const Velocity *vel)
{
    MotorSpeed w;
    float L = WHEEL_DISTANCE;
    float raw_D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    float raw_A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    float raw_B = vel->vx + L * vel->omega;
    bool is_strafe = (fabsf(vel->vx) > 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) < 0.001f);
    if (is_strafe) {
        w.D = raw_D * STRAFE_SCALE_D;
        w.A = raw_A * STRAFE_SCALE_A;
        w.B = raw_B * STRAFE_SCALE_B;
    } else {
        w.D = raw_D;
        w.A = raw_A;
        w.B = raw_B;
    }
    w.D = clampf(w.D, -(float)PWM_CAP, (float)PWM_CAP);
    w.A = clampf(w.A, -(float)PWM_CAP, (float)PWM_CAP);
    w.B = clampf(w.B, -(float)PWM_CAP, (float)PWM_CAP);
    return w;
}

static void apply_vel(const Velocity *vel)
{
    MotorSpeed s = inverse_kinematics(vel);
    g_last_vy = vel->vy;
    g_last_om = vel->omega;
    g_last_wheels = s;
    set_all_motors(&s);
}

static void strafe(float vx)
{
    Velocity v = { .vx = vx, .vy = 0.0f, .omega = 0.0f };
    apply_vel(&v);
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

    bool wide_l = false;
    bool wide_r = false;
    int cap = max_line_w();
    int near_bars = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        if (g_scan_rows[i].full_bar) {
            if (i < 3) {
                near_bars++;
            }
            int L = 0, R = 0;
            probe_sides(stem_cx, g_scan_y[i], &L, &R);
            if (L > 0) {
                wide_l = true;
            }
            if (R > 0) {
                wide_r = true;
            }
            if (L == 0 && R == 0) {
                wide_l = true;
                wide_r = true;
            }
        }
        for (int k = 0; k < g_scan_rows[i].n; k++) {
            if (g_scan_rows[i].b[k].width >= cap) {
                if (g_scan_rows[i].b[k].cx <= stem_cx) {
                    wide_l = true;
                }
                if (g_scan_rows[i].b[k].cx >= stem_cx) {
                    wide_r = true;
                }
            }
        }
    }
    /* 左或右有一大片，或两边都有：都算 T */
    if (wide_l || wide_r || near_bars >= 1) {
        s.t_bar = true;
    }

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
        s.t_bar = true;
        s.turn_left = false;
        s.turn_right = false;
    }

    if (!s.near_ok && s.has_black) {
        s.type = VIEW_BAR;
        if (s.t_bar) {
            /* T：不再当成单侧直角 */
        } else if (!s.turn_left && !s.turn_right) {
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
    if (g_phase == PHASE_STOP) {
        return "终止线：横着全黑后再全白，停车。";
    }
    if (g_phase == PHASE_ALIGN) {
        return "超声波已近，先原地摆正朝向再横移。";
    }
    if (g_phase == PHASE_STRAFE_FORCE || g_phase == PHASE_STRAFE_WAIT) {
        return "横移绕障。";
    }
    if (g_phase == PHASE_FWD) {
        return "绕过障碍后直行一段。";
    }
    if (g_phase == PHASE_STRAFE_BACK) {
        return "反向横移，直到再次看到黑线。";
    }
    if (p->t_bar) {
        return "T 路口：左或右有一大片横带（或两边都有）。绕障后见此则直行，全白停车。";
    }
    switch (p->type) {
    case VIEW_STEM:
        if (p->turn_left) {
            return "有竖线，左侧有横带或折线：记下左转，现在仍直行微调。";
        }
        if (p->turn_right) {
            return "有竖线，右侧有横带或折线：记下右转，现在仍直行微调。";
        }
        return "有竖线：慢速直行，按最低端偏角及时小调或大调。";
    case VIEW_BAR:
        return "窗口里是横带、还看不见竖线：记下方向，继续往前，等丢线再自转。";
    default:
        return "看不见黑线：按记忆方向原地转。";
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
    if ((++dbg_n & 1) != 0) {
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
                     "\"dist\":%.1f,\"tbar\":%d,\"roi0\":%d,\"roi1\":%d,\"ny\":%d,\"fy\":%d,"
                     "\"kx\":%d,\"ky\":%d,\"stem\":%d,\"kink\":%d,\"rows\":%d,"
                     "\"ldir\":\"%s\",\"hint\":\"%s\",\"scan_y\":[",
                     view_name(p->type), (int)g_phase, p->near_cx, p->far_cx, p->angle,
                     p->offset, p->left_mass, p->right_mass, g_last_vy, g_last_om, g_decode_ms,
                     g_last_dist, p->t_bar ? 1 : 0,
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
    "<!DOCTYPE html><html><head><meta charset=utf-8><title>摄像头画面</title>"
    "<style>body{font-family:sans-serif;background:#111;color:#ddd;margin:16px;max-width:1100px}"
    ".tip{background:#1e2a1e;border:1px solid #3a5;color:#cfc;padding:10px 12px;margin:0 0 14px}"
    ".row{display:flex;flex-wrap:wrap;gap:16px}"
    "figure{margin:0}"
    "canvas{background:#000;image-rendering:pixelated;width:min(90vw,640px);border:1px solid #444}"
    "figcaption{color:#aaa;margin:6px 0 0}"
    "pre{background:#1a1a1a;padding:10px;line-height:1.45;white-space:pre-wrap}"
    "ol{line-height:1.55;color:#ccc} li{margin:6px 0}</style></head><body>"
    "<h2>摄像头实时画面</h2>"
    "<p class=tip>手机/电脑连 WiFi <b>CAM_LINE</b> 密码 <b>12345678</b>，浏览器打开 "
    "<b>http://192.168.4.1/</b>（必须 http，不要 https）。左图就是摄像头看到的内容。</p>"
    "<div class=row>"
    "<figure><canvas id=a width=60 height=40></canvas>"
    "<figcaption>左：摄像头原图（已按车头方向翻转）。青框=循迹检测窗口</figcaption></figure>"
    "<figure><canvas id=b width=60 height=40></canvas>"
    "<figcaption>右：识别图。白=地面　黑=线　灰=不看。品红=竖线　绿=最低端</figcaption></figure>"
    "</div>"
    "<pre id=t>连接中...</pre>"
    "<h3>判断逻辑</h3>"
    "<ol>"
    "<li>慢速循迹。有竖线就直行，用绿点偏角及时小调/大调，把车身摆正，方便后面横移。</li>"
    "<li>超声波 ≤10cm：先原地摆正，再强制横移一小段；仍 &lt;13cm 继续横移；大于则直行一段，再反向横移直到看到黑线。</li>"
    "<li>终止 T：左或右任意一侧有一大片横带，或两边都有，都算 T。绕障回来后见 T 就直行，变成全白停车（不再当直角转）。</li>"
    "<li>直角/锐角：只记方向，丢线后再原地转。</li>"
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
    "const md=['循迹 FOLLOW','摆正 ALIGN','强制横移','继续横移','绕障直行','回线横移','停车 STOP'][inf.md]||inf.md;"
    "document.getElementById('t').textContent="
    "'判定：'+inf.type+'    模式：'+md+'    记忆方向：'+inf.ldir+"
    "'\\n超声波 dist='+inf.dist+'cm    终止T='+inf.tbar+"
    "'\\n最低端(绿) n='+inf.near+' 偏角 heading='+inf.heading+' 偏移 offset='+inf.offset+"
    "'\\nstem='+inf.stem+'  kink='+inf.kink+'  L/R='+inf.L+'/'+inf.R+"
    "'\\nvy='+inf.vy+' om='+inf.om+'  解码 '+inf.ms+'ms'"
    "+'\\n\\n'+inf.hint;"
    "}catch(e){document.getElementById('t').textContent='等待画面 '+e;}"
    "busy=false;}"
    "setInterval(tick,300);tick();"
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
    ESP_LOGI(TAG, "调试页: 连 WiFi CAM_LINE 密码 12345678，浏览器打开 http://192.168.4.1/");
}

static void follow_stem(const Sight *p)
{
    int dead = scaled_px(DEAD_PX_480);
    int macro = scaled_px(MACRO_PX_480);
    int err = p->angle;
    if (abs(err) <= dead) {
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

static bool heading_ok(const Sight *p)
{
    int dead = scaled_px(DEAD_PX_480);
    int off = scaled_px(14);
    return p->near_ok && abs(p->angle) <= dead && abs(p->offset) <= off;
}

static void begin_phase(Phase ph)
{
    g_phase = ph;
    g_phase_t0 = esp_timer_get_time();
}

static void align_heading(const Sight *p)
{
    int err = p->near_ok ? ((abs(p->angle) > 0) ? p->angle : p->offset) : 0;
    if (!p->near_ok) {
        spin_in_place(g_last_dir == LAST_DIR_LEFT);
        return;
    }
    if (abs(err) <= scaled_px(DEAD_PX_480)) {
        drive(0.0f, 0.0f);
        return;
    }
    /* 不前进时差速太小转不动，改用原地转摆正 */
    spin_in_place(err < 0);
}

static void follow_tick(const Sight *p)
{
    if (g_has_avoided && (p->t_bar || p->type == VIEW_BAR)) {
        g_t_seen = true;
        g_t_white = 0;
        drive(BASE_SPEED, 0.0f);
        return;
    }
    /* 避障前：只有左右都是横带的十字才当 T 直行；单侧仍记直角 */
    if (!g_has_avoided && p->t_bar && !p->turn_left && !p->turn_right) {
        drive(BASE_SPEED, 0.0f);
        return;
    }
    if (p->turn_left) {
        g_last_dir = LAST_DIR_LEFT;
    } else if (p->turn_right) {
        g_last_dir = LAST_DIR_RIGHT;
    }

    if (g_t_seen) {
        if (p->type == VIEW_NONE) {
            g_t_white++;
            if (g_t_white >= T_WHITE_FRAMES) {
                stop_motors();
                begin_phase(PHASE_STOP);
                ESP_LOGI(TAG, "终止线：全黑后全白，停车");
                return;
            }
            drive(BASE_SPEED, 0.0f);
            return;
        }
        g_t_white = 0;
        drive(BASE_SPEED, 0.0f);
        return;
    }

    if (p->type == VIEW_NONE) {
        g_lost_frames++;
        if (g_lost_frames > LOST_STOP_FRAMES) {
            stop_motors();
            ESP_LOGE(TAG, "连续丢线，停车保护");
            return;
        }
        spin_in_place(g_last_dir == LAST_DIR_LEFT);
        return;
    }

    g_lost_frames = 0;
    if (p->type == VIEW_STEM) {
        follow_stem(p);
    } else {
        drive(BASE_SPEED, 0.0f);
    }
}

static void control_once(void)
{
    Sight p = look();
    float dist = ultrasonic_cm();

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG, "ph=%d %s stem=%d ang=%d off=%d dist=%.1f t=%d vy=%.0f om=%.0f %dms",
                 (int)g_phase, view_name(p.type), p.stem_n, p.angle, p.offset,
                 dist, (int)g_t_seen, g_last_vy, g_last_om, g_decode_ms);
        last_log = now;
    }

    switch (g_phase) {
    case PHASE_FOLLOW:
        if (!g_t_seen && dist > 0.0f && dist <= AVOID_TRIGGER_CM) {
            g_hit_cm++;
        } else {
            g_hit_cm = 0;
        }
        if (g_hit_cm >= 2) {
            ESP_LOGW(TAG, "障碍 %.1f cm，先摆正再横移", dist);
            g_hit_cm = 0;
            begin_phase(PHASE_ALIGN);
            align_heading(&p);
            break;
        }
        follow_tick(&p);
        break;

    case PHASE_ALIGN:
        if (heading_ok(&p) || (now - g_phase_t0) > (int64_t)STRAFE_ALIGN_MS * 1000) {
            ESP_LOGI(TAG, "开始横移 (朝向%s)", heading_ok(&p) ? "已正" : "超时");
            begin_phase(PHASE_STRAFE_FORCE);
            strafe(STRAFE_SPEED);
        } else {
            align_heading(&p);
        }
        break;

    case PHASE_STRAFE_FORCE:
        strafe(STRAFE_SPEED);
        if ((now - g_phase_t0) > (int64_t)STRAFE_FORCE_MS * 1000) {
            begin_phase(PHASE_STRAFE_WAIT);
        }
        break;

    case PHASE_STRAFE_WAIT:
        strafe(STRAFE_SPEED);
        if (dist < 0.0f || dist >= AVOID_CLEAR_CM) {
            ESP_LOGI(TAG, "前方已空 %.1f cm，直行绕过", dist);
            begin_phase(PHASE_FWD);
            drive(BASE_SPEED, 0.0f);
        }
        break;

    case PHASE_FWD:
        drive(BASE_SPEED, 0.0f);
        if ((now - g_phase_t0) > (int64_t)AVOID_FWD_MS * 1000) {
            ESP_LOGI(TAG, "反向横移找线");
            begin_phase(PHASE_STRAFE_BACK);
            strafe(-STRAFE_BACK_SPEED);
        }
        break;

    case PHASE_STRAFE_BACK:
        strafe(-STRAFE_BACK_SPEED);
        if (p.has_black || p.near_ok) {
            stop_motors();
            ESP_LOGI(TAG, "重新看到黑线，恢复循迹");
            g_has_avoided = true;
            g_lost_frames = 0;
            begin_phase(PHASE_FOLLOW);
        } else if ((now - g_phase_t0) > 4000000) {
            ESP_LOGW(TAG, "回线超时，停车");
            stop_motors();
            begin_phase(PHASE_STOP);
        }
        break;

    case PHASE_STOP:
        stop_motors();
        if (g_mission == MISSION_LINE) {
            g_mission = MISSION_WAIT;
            g_wait_t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "循迹结束，等待 3 秒后推球");
        }
        break;
    }

    debug_update(&p);
}


/* ========== catch.c 推球（符号已改名避免冲突） ========== */
#define APPROACH_SPEED  40.0f   /* 点动脉冲幅值，不是连续走 */
#define PUSH_SPEED      64.0f   /* 快速撞球 */
#define BACKUP_SPEED    36.0f   /* 匀速倒退 */
#define CATCH_OMEGA_MICRO  12.0f
#define CATCH_OMEGA_MACRO  18.0f
#define CATCH_SPIN_PWM   42.0f
#define CATCH_ROT_MAX     22.0f
/* 仅 ST_ALIGN 两个 pivot 使用，与 orbit_test.c 一致，勿改搜球/对中宏 */
#define CATCH_ORBIT_SIN60       0.8660254f
#define CATCH_ORBIT_SCALE_D     0.4f
#define CATCH_ORBIT_SCALE_A     0.4f
#define CATCH_ORBIT_SCALE_B     3.0f
#define CATCH_ORBIT_B_BOOST     1.00f
#define CATCH_ORBIT_VX          66.0f
#define CATCH_ORBIT_PULSE_MS    70
#define CATCH_ORBIT_BRAKE_MS    5       /* 仅绕球点动；勿改搜球制动 */
#define PULSE_ON_MS     120     /* 点动时长 ×4 */
#define PULSE_SPIN_ON_MS 20     /* 搜球/对中：真 20ms 后立刻停，再看下一帧 */
#define PULSE_BACKUP_ON_MS 220  /* 后退每下比靠近走得更远 */
#define PULSE_OFF_MS    80      /* 前进/后退点动间隔；搜球不等这段 */
#define PULSE_BRAKE_SCALE 0.32f /* 反向制动幅值，低于起步静摩擦 */
#define PULSE_BRAKE_MAX_MS 24   /* 制动必须远短于正向，避免反走 */
#define PUSH_MS         500     /* 快速撞击时长 */
#define BACKUP_MS       3000    /* 匀速倒退时长 */
#define APPROACH_STOP_CM 8.0f

#define STAGE_TIMEOUT_US    (18LL * 1000 * 1000)
#define LOCK_HOLD_FRAMES    4
#define STOP_HOLD_FRAMES    3
#define ALIGN_HOLD_FRAMES   5
#define COINCIDE_PX         8
#define CENTER_DEAD_PX      6
#define CENTER_MACRO_PX     18
#define ALIGN_VERT_PX       4       /* 解码图上球-网 cx 差须同时满足 */
#define ALIGN_OK_DEG        10.0f   /* 与 ALIGN_VERT_PX 同时满足才快射 */
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
static FrameSight g_sight;
static BlobTarget g_locked_ball;
static bool g_done_red = false;
static bool g_done_blue = false;
static float g_last_d_ball = -1.0f;
static float g_last_d_net = -1.0f;
static bool g_orbit_dir_valid = false;
static bool g_orbit_left = true;
static int g_last_abs_ldx = -1;
static int g_orbit_worse_frames = 0;
static FrameSight g_dbg_sight;
static PushState g_dbg_state;
static BallKind g_dbg_kind;

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

static MotorSpeed catch_make_drive(float vy, float omega)
{
    float fwd = 0.0f;
    if (fabsf(vy) > 1.0f) {
        fwd = clampf(vy, -(float)PWM_CAP, (float)PWM_CAP);
    }
    float rot = clampf(omega, -CATCH_ROT_MAX, CATCH_ROT_MAX);

    MotorSpeed s;
    s.D = clampf(fwd + rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.A = clampf(fwd - rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.B = clampf(rot * 0.70f, -(float)PWM_CAP, (float)PWM_CAP);
    g_last_vy = vy;
    g_last_om = rot;
    return s;
}

static void catch_drive(float vy, float omega)
{
    MotorSpeed s = catch_make_drive(vy, omega);
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
        vTaskDelay(0);
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
    MotorSpeed s = catch_make_drive(vy, omega);
    pulse_apply(&s);
}

static void pulse_drive_ms(float vy, float omega, int on_ms)
{
    MotorSpeed s = catch_make_drive(vy, omega);
    pulse_apply_ms(&s, on_ms, PULSE_OFF_MS);
}

static void catch_spin_in_place(bool left)
{
    float s = left ? -CATCH_SPIN_PWM : CATCH_SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    /* 转完立刻停，下一帧再判断，避免连转大半圈 */
    pulse_apply_ms(&m, PULSE_SPIN_ON_MS, 0);
}

/* 绕车头前方点公转，配速与 orbit_test.c 的 catch_orbit_cmd 相同 */
static MotorSpeed catch_orbit_cmd(bool left)
{
    float vx = left ? CATCH_ORBIT_VX : -CATCH_ORBIT_VX;
    MotorSpeed m;
    m.D = clampf(-CATCH_ORBIT_SIN60 * vx * CATCH_ORBIT_SCALE_D, -(float)PWM_CAP, (float)PWM_CAP);
    m.A = clampf( CATCH_ORBIT_SIN60 * vx * CATCH_ORBIT_SCALE_A, -(float)PWM_CAP, (float)PWM_CAP);
    m.B = clampf(vx * CATCH_ORBIT_SCALE_B * CATCH_ORBIT_B_BOOST,
                 -(float)PWM_CAP, (float)PWM_CAP);
    return m;
}

/* 仅 ST_ALIGN：70ms 通电 + 固定 5ms 反刹，不走 pulse_apply_ms */
static void pulse_orbit_around_front(bool left)
{
    MotorSpeed m = catch_orbit_cmd(left);
    g_last_vy = 0.0f;
    g_last_om = m.B;
    g_last_wheels = m;

    int64_t now = esp_timer_get_time();
    if (now < g_pulse_ready_at) {
        stop_motors();
        return;
    }

    int on_ms = CATCH_ORBIT_PULSE_MS;
    if (on_ms < 1) {
        on_ms = 1;
    }
    set_all_motors(&m);
    wait_pulse_us((int64_t)on_ms * 1000);

    MotorSpeed brk = pulse_brake_cmd(&m);
    set_all_motors(&brk);
    wait_pulse_us((int64_t)CATCH_ORBIT_BRAKE_MS * 1000);

    stop_motors();
    g_last_wheels = m;
    g_pulse_ready_at = 0;
}

/* |ldx| 变小则保持原方向；连续变大才按 ldx 符号改向 */
static bool align_orbit_left(int ldx)
{
    int al = abs(ldx);
    bool want_left = (ldx < 0);
    if (!g_orbit_dir_valid) {
        g_orbit_left = want_left;
        g_orbit_dir_valid = true;
        g_orbit_worse_frames = 0;
    } else if (g_last_abs_ldx >= 0) {
        if (al < g_last_abs_ldx) {
            g_orbit_worse_frames = 0;
        } else if (al > g_last_abs_ldx) {
            g_orbit_worse_frames++;
            if (g_orbit_worse_frames >= 2) {
                g_orbit_left = want_left;
                g_orbit_worse_frames = 0;
            }
        }
    }
    g_last_abs_ldx = al;
    return g_orbit_left;
}

static void enter_state(PushState st)
{
    if (g_state != st) {
        ESP_LOGI(TAG, "STATE %s -> %s  ball=%s",
                 state_name(g_state), state_name(st), ball_name(g_ball_kind));
        g_pulse_ready_at = 0;
        g_stop_hold = 0;
        g_lock_frames = 0;
        if (st == ST_ALIGN) {
            g_orbit_dir_valid = false;
            g_last_abs_ldx = -1;
            g_orbit_worse_frames = 0;
        }
    }
    g_state = st;
    g_stage_t0 = esp_timer_get_time();
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

/* UVC MJPEG 常省略 DHT。彩色走 esp_jpeg（软件 tjpgd + 默认 Huffman）。
 * 循迹灰度走 cam_jd_prepare，不能和 esp_jpeg 共用 jd_prepare 符号。 */
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

/* 本任务必须先 esp_task_wdt_add。vTaskDelay(0) 让不出 IDLE，喂不了 IDLE0 看门狗。 */
static void pet_wdt(void)
{
    (void)esp_task_wdt_reset();
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

    if (!s_rgb || !s_jpeg_work) {
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
        .advanced = {
            .working_buffer = s_jpeg_work,
            .working_buffer_size = JPEG_RGB_WORK_SZ,
        },
    };
    esp_jpeg_image_output_t out = {0};

    pet_wdt();
    vTaskDelay(1);
    esp_err_t err = esp_jpeg_decode(&cfg, &out);
    pet_wdt();
    vTaskDelay(1);
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
        if ((y & 7) == 0) {
            pet_wdt();
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
        if ((area & 511) == 0) {
            pet_wdt();
        }
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
            if ((y & 7) == 0) {
                pet_wdt();
            }
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
        dilate_mask(1);
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
        if ((y & 7) == 0) {
            pet_wdt();
        }
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

/* 在已通过绿网形状过滤的块里，选和球同一侧且 |cx| 最近的门 */
static bool pick_net_among(const BlobTarget *cands, int n, const BlobTarget *ball,
                           BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    if (n <= 0) {
        return false;
    }
    if (!ball || !ball->found) {
        int best_score = -1;
        for (int i = 0; i < n; i++) {
            int score = cands[i].area * (cands[i].mean_v + 1);
            if (!best->found || score > best_score) {
                *best = cands[i];
                best_score = score;
            }
        }
        return best->found;
    }

    int mid = s_img_w / 2;
    bool prefer_left = (ball->cx < mid);
    int best_i = -1;
    int best_dx = 99999;
    int best_cy = 99999;

    for (int pass = 0; pass < 2 && best_i < 0; pass++) {
        for (int i = 0; i < n; i++) {
            bool same_half = prefer_left ? (cands[i].cx < mid) : (cands[i].cx >= mid);
            if (pass == 0 && !same_half) {
                continue;
            }
            int dx = abs(cands[i].cx - ball->cx);
            if (dx < best_dx || (dx == best_dx && cands[i].cy < best_cy)) {
                best_dx = dx;
                best_cy = cands[i].cy;
                best_i = i;
            }
        }
    }
    if (best_i < 0) {
        return false;
    }
    *best = cands[best_i];
    return true;
}

static bool find_net_for_ball(const BlobTarget *ball, BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    build_mask(COLOR_GREEN, 0);
    dilate_mask(1);

    BlobTarget cands[MAX_CC_BLOBS];
    int n = 0;
    for (int y = 0; y < s_img_h; y++) {
        if ((y & 7) == 0) {
            pet_wdt();
        }
        for (int x = 0; x < s_img_w; x++) {
            size_t i = (size_t)y * s_img_w + x;
            if (!s_mask[i] || s_visited[i]) {
                continue;
            }
            BlobTarget cand;
            memset(&cand, 0, sizeof(cand));
            if (!flood_blob(x, y, &cand)) {
                continue;
            }
            if (cand.area < MIN_NET_AREA || cand.area > MAX_NET_AREA) {
                continue;
            }
            int bw = cand.x1 - cand.x0 + 1;
            int bh = cand.y1 - cand.y0 + 1;
            float ar = (bw < bh) ? (float)bw / (float)bh : (float)bh / (float)bw;
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
            if (n < MAX_CC_BLOBS) {
                cands[n++] = cand;
            }
            if (n >= MAX_CC_BLOBS) {
                goto picked;
            }
        }
    }
picked:
    return pick_net_among(cands, n, ball, best);
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
    return (fabsf(p->align_deg) <= ALIGN_OK_DEG) && (abs(ldx) <= ALIGN_VERT_PX);
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
    pet_wdt();
    vTaskDelay(1);
    find_best_blob(COLOR_BLUE, true, MIN_BALL_AREA, MAX_BALL_AREA, &out->blue);
    pet_wdt();
    vTaskDelay(1);

    if (g_state == ST_SEARCH_BALL || g_state == ST_IDLE) {
        pick_search_target(out);
    } else if (g_ball_kind == BALL_RED) {
        out->ball = out->red;
    } else {
        out->ball = out->blue;
    }

    pet_wdt();
    vTaskDelay(1);
    find_net_for_ball(out->ball.found ? &out->ball : NULL, &out->net);
    if (g_state == ST_SEARCH_BLACK || g_state == ST_RETURN_END) {
        out->has_black_line = detect_black_line(&out->black_cx);
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

static bool ball_on_center(const FrameSight *p)
{
    if (!p || !p->ball.found) {
        return false;
    }
    return abs(p->ball.cx - s_img_w / 2) <= center_dead_px() + 2;
}

static bool ready_to_shoot(const FrameSight *p)
{
    return line_almost_vertical(p) && ball_on_center(p);
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
        catch_spin_in_place(err < 0);
        return;
    }
    if (abs(err) <= dead) {
        pulse_drive(speed, 0.0f);
        return;
    }
    float om = (abs(err) <= macro) ? CATCH_OMEGA_MICRO : CATCH_OMEGA_MACRO;
}

static bool stage_timeout(void)
{
    return (esp_timer_get_time() - g_stage_t0) > STAGE_TIMEOUT_US;
}

/* ==================== 调试可视化（对齐 cam_line_follow） ==================== */
static void catch_debug_copy_images(void)
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

static void catch_debug_update(const FrameSight *p)
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
    catch_debug_copy_images();

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
    catch_spin_in_place(true);
}

static void catch_control_once(void)
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
        catch_debug_update(p);
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
            catch_spin_in_place(true);
        }
        break;

    case ST_APPROACH_BALL: {
        if (!p->ball.found) {
            enter_state(ST_SEARCH_BALL);
            catch_spin_in_place(true);
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
                if (ready_to_shoot(p)) {
                    ESP_LOGI(TAG, "连线 %.1f° 已对准且球居中，快速撞球 %.0fms", p->align_deg, (float)PUSH_MS);
                    enter_state(ST_PUSH);
                    catch_drive(PUSH_SPEED, 0.0f);
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
            catch_spin_in_place(true);
            break;
        }
        if (!p->net.found) {
            catch_spin_in_place(true);
            break;
        }
        int ldx = p->net.cx - p->ball.cx;
        if (ready_to_shoot(p)) {
            stop_motors();
            g_lock_frames++;
            g_orbit_dir_valid = false;
            if (g_lock_frames >= ALIGN_HOLD_FRAMES) {
                ESP_LOGI(TAG, "连线 %.1f° ldx=%d 球居中，快速撞球 %.0fms",
                         p->align_deg, ldx, (float)PUSH_MS);
                enter_state(ST_PUSH);
                catch_drive(PUSH_SPEED, 0.0f);
            }
        } else {
            g_lock_frames = 0;
            if (line_almost_vertical(p) && !ball_on_center(p)) {
                pulse_center_on_x(p->ball.cx, 0.0f);
            } else {
                pulse_orbit_around_front(align_orbit_left(ldx));
            }
        }
        break;
    }

    case ST_PUSH: {
        if (esp_timer_get_time() - g_stage_t0 >= (int64_t)PUSH_MS * 1000) {
            ESP_LOGI(TAG, "撞击 %.0fms 结束，匀速倒退 %.0fms", (float)PUSH_MS, (float)BACKUP_MS);
            on_push_success();
        } else {
            catch_drive(PUSH_SPEED, 0.0f);
        }
        break;
    }

    case ST_BACKUP:
        if (esp_timer_get_time() >= g_backup_until) {
            after_backup();
        } else {
            catch_drive(-BACKUP_SPEED, 0.0f);
        }
        break;

    case ST_SEARCH_BLACK:
    case ST_RETURN_END:
        enter_state(ST_SEARCH_BALL);
        catch_spin_in_place(true);
        break;

    case ST_DONE:
        stop_motors();
        g_mission = MISSION_DONE;
        break;

    case ST_FAIL:
        stop_motors();
        ESP_LOGW(TAG, "从 FAIL 恢复，重新搜球");
        enter_state(ST_SEARCH_BALL);
        catch_spin_in_place(true);
        break;

    default:
        enter_state(ST_FAIL);
        break;
    }

    catch_debug_update(p);
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
    s_rgb = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT * 3);
    s_mask = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_visited = (uint8_t *)psram_alloc(CAM_WIDTH * CAM_HEIGHT);
    s_jpeg_work = (uint8_t *)psram_alloc(JPEG_RGB_WORK_SZ);

    if (!xfer_a || !xfer_b || !frame_buf || !s_jpeg || !s_gray || !s_bin ||
        !s_rgb || !s_mask || !s_visited || !s_jpeg_work) {
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

    ESP_LOGI(TAG, "全程：循迹避障 -> 等 3 秒 -> 找球推球");
    if (esp_task_wdt_add(NULL) != ESP_OK) {
        ESP_LOGW(TAG, "line_follow 未能加入任务看门狗");
    }
    while (1) {
        pet_wdt();
        if (xSemaphoreTake(s_frame_ready, pdMS_TO_TICKS(150)) != pdTRUE) {
            if (g_mission == MISSION_CATCH) {
                if (g_state == ST_SEARCH_BALL || g_state == ST_IDLE) {
                    catch_spin_in_place(true);
                }
            } else if (g_mission != MISSION_DONE) {
                g_lost_frames++;
                if (g_lost_frames > LOST_STOP_FRAMES) {
                    stop_motors();
                }
            } else {
                stop_motors();
            }
            vTaskDelay(pdMS_TO_TICKS(5));
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
            vTaskDelay(1);
            continue;
        }

        if (g_mission == MISSION_WAIT) {
            stop_motors();
            if ((esp_timer_get_time() - g_wait_t0) >= (int64_t)CATCH_WAIT_MS * 1000) {
                ESP_LOGI(TAG, "等待结束，开始找球推球");
                g_mission = MISSION_CATCH;
                enter_state(ST_SEARCH_BALL);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int64_t t0 = esp_timer_get_time();
        if (g_mission == MISSION_CATCH || g_mission == MISSION_DONE) {
            if (!decode_mjpeg_rgb(local_jpg, (int)len)) {
                ESP_LOGW(TAG, "JPEG 彩色解码失败 len=%u", (unsigned)len);
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
            /* 让 IDLE0 / 中断看门狗有机会跑，避免第一帧彩色处理把芯片复位 */
            pet_wdt();
            vTaskDelay(pdMS_TO_TICKS(15));
            if (g_mission == MISSION_CATCH) {
                catch_control_once();
            } else {
                stop_motors();
            }
            pet_wdt();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (!decode_mjpeg(local_jpg, (int)len)) {
            ESP_LOGW(TAG, "JPEG 解码失败 len=%u", (unsigned)len);
            vTaskDelay(1);
            continue;
        }
        g_decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        control_once();
        vTaskDelay(1);
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
    ultrasonic_init();
    encoder_init();
    stop_motors();
    lcd_init_and_ui();
    wifi_debug_start();

    if (!camera_start()) {
        ESP_LOGE(TAG, "摄像头初始化失败");
        return;
    }

    xTaskCreatePinnedToCore(gui_task, "gui_task", 6144, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(line_task, "line_follow", 16384, NULL, 6, NULL, 0);
}
