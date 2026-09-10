#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

// LVGL & LCD 驱动头文件
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

static const char* TAG = "MAIN_APP";

// ==================== 1. 引脚与硬件配置 ====================

// --- 循迹红外传感器 ---
#define SENSOR_L2_PIN   GPIO_NUM_4
#define SENSOR_L1_PIN   GPIO_NUM_5
#define SENSOR_R1_PIN   GPIO_NUM_6
#define SENSOR_R2_PIN   GPIO_NUM_7

// --- 电机 PWM & 方向控制 ---
// 左前 (D)
#define MOTOR_D_PWM     GPIO_NUM_14
#define MOTOR_D_IN1     GPIO_NUM_13
#define MOTOR_D_IN2     GPIO_NUM_12
// 右前 (A)
#define MOTOR_A_PWM     GPIO_NUM_21
#define MOTOR_A_IN1     GPIO_NUM_46
#define MOTOR_A_IN2     GPIO_NUM_3
// 后轮 (B)
#define MOTOR_B_PWM     GPIO_NUM_15
#define MOTOR_B_IN1     GPIO_NUM_16
#define MOTOR_B_IN2     GPIO_NUM_17

// --- 霍尔编码器 (512线AB相) ---
#define HALL_D_A_PIN    GPIO_NUM_10      // 左前轮霍尔A相
#define HALL_D_B_PIN    GPIO_NUM_11      // 左前轮霍尔B相
#define HALL_A_A_PIN    GPIO_NUM_20      // 右前轮霍尔A相
#define HALL_A_B_PIN    GPIO_NUM_19      // 右前轮霍尔B相
#define HALL_B_A_PIN    GPIO_NUM_42      // 后轮霍尔A相
#define HALL_B_B_PIN    GPIO_NUM_41      // 后轮霍尔B相

// --- 超声波传感器 ---
#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000            // 30ms 超时

// --- LED 显示屏 (ST7789/ST7735 128x160) ---
#define PIN_NUM_CS      GPIO_NUM_2      // 片选
#define PIN_NUM_SCK     GPIO_NUM_1      // 时钟
#define PIN_NUM_SDI     GPIO_NUM_38     // MOSI 数据
#define PIN_NUM_DC      GPIO_NUM_39     // D/C 命令/数据
#define PIN_NUM_RST     GPIO_NUM_40     // 复位

#define LCD_H_RES       128
#define LCD_V_RES       160

// ==================== 2. PWM 与 运动控制参数 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        1000
#define PWM_RESOL       8
#define MAX_SPEED       255

#define BASE_SPEED      85      // 基础速度
#define STRAFE_SPEED    47      // 横向平移速度
#define CONTROL_PERIOD  3       // 循迹周期 (ms)
#define SEARCH_PERIOD   8      // 寻线周期 (ms)

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f

#define MICRO_PULSE_MS  2
#define MACRO_PULSE_MS  2.5
#define OMEGA_MICRO     15.0f
#define OMEGA_MACRO     18.0f
#define OMEGA_SEARCH    310.0f

// 轮速 Scale 调节参数
#define STRAFE_SCALE_D   0.87f     
#define STRAFE_SCALE_A   0.87f     
#define STRAFE_SCALE_B   1.20f     

#define FWD_SCALE_D      1.00f
#define FWD_SCALE_A      1.00f
#define FWD_SCALE_B      1.00f

#define ROT_SCALE_D      1.00f
#define ROT_SCALE_A      1.00f
#define ROT_SCALE_B      1.00f

#define ENCODER_PPR      512      // 编码器线数
#define SAMPLE_TIME_MS   100      // 测速采样周期 (ms)

#define WHEEL_D 0  // 左前
#define WHEEL_A 1  // 右前
#define WHEEL_B 2  // 后轮

// ==================== 3. 数据结构与全局变量 ====================
typedef struct {
    int l2, l1, r1, r2;
} SensorData;

typedef struct {
    float D, A, B;
} MotorSpeed;

typedef struct {
    float vx, vy, omega;
} Velocity;

typedef enum {
    LAST_DIR_LEFT,
    LAST_DIR_RIGHT
} LastDirection;

static LastDirection g_last_dir = LAST_DIR_LEFT;
static bool g_has_avoided = false; 

// 编码器结构体
typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel;
    int32_t pulse_count;      
    int32_t last_count;       
} EncoderHandle_t;

typedef struct {
    float rpm_D;  
    float rpm_A;  
    float rpm_B;  
} AllWheelRPM;

static EncoderHandle_t g_encoders[3] = {0};

// --- LVGL 全局 UI 控件句柄 ---
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf1[LCD_H_RES * 20];
static lv_color_t buf2[LCD_H_RES * 20];

static lv_obj_t *lbl_wheel_a = NULL;
static lv_obj_t *lbl_wheel_b = NULL;
static lv_obj_t *lbl_wheel_d = NULL;
static lv_obj_t *lbl_dist    = NULL;

// 全局线程安全的数据共享变量（用于UI更新）
static AllWheelRPM g_current_rpm = {0};
static float g_current_distance = 0.0f;

// ==================== 4. LVGL 与屏幕底层驱动 ====================
static void increase_lvgl_tick(void *arg) {
    lv_tick_inc(2); // 每 2ms 增加一次 tick
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

void lcdInitAndUiSetup(void) {
    ESP_LOGI(TAG, "初始化 LCD SPI 总线...");
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

    // 初始化 LVGL
    lv_init();
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LCD_H_RES * 20);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
    lv_disp_drv_register(&disp_drv);

    // 设置高精度心跳
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    // 构建 UI 视图
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_white(), LV_PART_MAIN);

    lv_obj_t *cont = lv_obj_create(lv_scr_act());
    lv_obj_set_size(cont, 120, 150);
    lv_obj_center(cont);
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_pad_all(cont, 2, 0);

    lbl_wheel_a = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_a, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_a, "WheelA: 0.0");

    lbl_wheel_b = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_b, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_b, "WheelB: 0.0");

    lbl_wheel_d = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_d, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_wheel_d, "WheelD: 0.0");

    lbl_dist = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_dist, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(lbl_dist, "US Dist: 0.0cm");
}

// ==================== 5. 电机/传感器/编码器硬件驱动 ====================
void motorInit(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_D_PWM) | (1ULL << MOTOR_D_IN1) | (1ULL << MOTOR_D_IN2) |
                        (1ULL << MOTOR_A_PWM) | (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_PWM) | (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);
    
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = PWM_RESOL,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);
    
    ledc_channel_config_t ch_conf = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
    };
    ch_conf.channel = LEDC_CH_D; ch_conf.gpio_num = MOTOR_D_PWM; ledc_channel_config(&ch_conf);
    ch_conf.channel = LEDC_CH_A; ch_conf.gpio_num = MOTOR_A_PWM; ledc_channel_config(&ch_conf);
    ch_conf.channel = LEDC_CH_B; ch_conf.gpio_num = MOTOR_B_PWM; ledc_channel_config(&ch_conf);
}

void setMotor(gpio_num_t pwm, gpio_num_t in1, gpio_num_t in2, float speed, ledc_channel_t channel) {
    int pwm_val = (int)round(speed);
    if (pwm_val > MAX_SPEED) pwm_val = MAX_SPEED;
    if (pwm_val < -MAX_SPEED) pwm_val = -MAX_SPEED;

    if (pwm_val > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, pwm_val);
    } else if (pwm_val < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        ledc_set_duty(LEDC_MODE, channel, -pwm_val);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, channel, 0);
    }
    ledc_update_duty(LEDC_MODE, channel);
}

void setAllMotors(const MotorSpeed* speed) {
    setMotor(MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, speed->D, LEDC_CH_D);
    setMotor(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, speed->A, LEDC_CH_A);
    setMotor(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, speed->B, LEDC_CH_B);
}

void stopMotors(void) {
    MotorSpeed zero = {0, 0, 0};
    setAllMotors(&zero);
}

void sensorInit(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_L2_PIN) | (1ULL << SENSOR_L1_PIN) |
                        (1ULL << SENSOR_R1_PIN) | (1ULL << SENSOR_R2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
}

void readSensors(SensorData* data) {
    data->l2 = gpio_get_level(SENSOR_L2_PIN);
    data->l1 = gpio_get_level(SENSOR_L1_PIN);
    data->r1 = gpio_get_level(SENSOR_R1_PIN);
    data->r2 = gpio_get_level(SENSOR_R2_PIN);
}

int getSensorCode(const SensorData* data) {
    int s_l2 = (data->l2 == 0) ? 1 : 0;
    int s_l1 = (data->l1 == 0) ? 1 : 0;
    int s_r1 = (data->r1 == 0) ? 1 : 0;
    int s_r2 = (data->r2 == 0) ? 1 : 0;
    return (s_l2 << 3) | (s_l1 << 2) | (s_r1 << 1) | s_r2;
}

void ultrasonicInit(void) {
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&echo_conf);

    gpio_set_level(TRIG_PIN, 0);
}

float getUltrasonicDistance(void) {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_wait > TIMEOUT_US) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > TIMEOUT_US) return -2.0f;
    }

    int64_t echo_end = esp_timer_get_time();
    float dist = (float)(echo_end - echo_start) * 0.0343f / 2.0f;
    
    if (dist > 0) g_current_distance = dist; 
    return dist;
}

void init_single_encoder(gpio_num_t pin_a, gpio_num_t pin_b, EncoderHandle_t* handle) {
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

    handle->pulse_count = 0;
    handle->last_count = 0;
}

void encoderInit(void) {
    init_single_encoder(HALL_D_A_PIN, HALL_D_B_PIN, &g_encoders[WHEEL_D]);
    init_single_encoder(HALL_A_A_PIN, HALL_A_B_PIN, &g_encoders[WHEEL_A]);
    init_single_encoder(HALL_B_A_PIN, HALL_B_B_PIN, &g_encoders[WHEEL_B]);
}

AllWheelRPM getAllWheelRPM(void) {
    AllWheelRPM result = {0.0f, 0.0f, 0.0f};

    static int64_t last_time = 0;
    static int32_t last_count_D = 0, last_count_A = 0, last_count_B = 0;

    int64_t now = esp_timer_get_time();

    if (last_time == 0) {
        pcnt_unit_get_count(g_encoders[WHEEL_D].unit, (int*)&last_count_D);
        pcnt_unit_get_count(g_encoders[WHEEL_A].unit, (int*)&last_count_A);
        pcnt_unit_get_count(g_encoders[WHEEL_B].unit, (int*)&last_count_B);
        last_time = now;
        return result;
    }

    int64_t elapsed_us = now - last_time;
    if (elapsed_us < SAMPLE_TIME_MS * 1000) return g_current_rpm;

    int count_D = 0, count_A = 0, count_B = 0;
    pcnt_unit_get_count(g_encoders[WHEEL_D].unit, &count_D);
    pcnt_unit_get_count(g_encoders[WHEEL_A].unit, &count_A);
    pcnt_unit_get_count(g_encoders[WHEEL_B].unit, &count_B);

    int32_t delta_D = count_D - last_count_D;
    int32_t delta_A = count_A - last_count_A;
    int32_t delta_B = count_B - last_count_B;

    last_count_D = count_D; last_count_A = count_A; last_count_B = count_B;

    float minutes = (float)elapsed_us / 60000000.0f;
    result.rpm_D = ((float)delta_D / ENCODER_PPR) / minutes;
    result.rpm_A = ((float)delta_A / ENCODER_PPR) / minutes;
    result.rpm_B = ((float)delta_B / ENCODER_PPR) / minutes;

    last_time = now;
    g_current_rpm = result; 
    return result;
}

// ==================== 6. 运动学逆解 & 循迹控制逻辑 ====================
MotorSpeed inverseKinematics(const Velocity* vel) {
    MotorSpeed wheels;
    float L = WHEEL_DISTANCE;
    
    float raw_D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    float raw_A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    float raw_B =  vel->vx + L * vel->omega;

    bool is_strafe  = (fabsf(vel->vx) > 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_forward = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) > 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_rotate  = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) > 0.001f);

    if (is_strafe) {
        wheels.D = raw_D * STRAFE_SCALE_D;
        wheels.A = raw_A * STRAFE_SCALE_A;
        wheels.B = raw_B * STRAFE_SCALE_B;
    } else if (is_forward) {
        wheels.D = raw_D * FWD_SCALE_D;
        wheels.A = raw_A * FWD_SCALE_A;
        wheels.B = raw_B * FWD_SCALE_B;
    } else if (is_rotate) {
        wheels.D = raw_D * ROT_SCALE_D;
        wheels.A = raw_A * ROT_SCALE_A;
        wheels.B = raw_B * ROT_SCALE_B;
    } else {
        wheels.D = raw_D; wheels.A = raw_A; wheels.B = raw_B;
    }
    return wheels;
}

void executePulse(Velocity vel, int duration_ms) {
    MotorSpeed speed = inverseKinematics(&vel);
    setAllMotors(&speed);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

bool controlLoop(int baseSpeed) {
    SensorData data;
    readSensors(&data);
    int code = getSensorCode(&data);

    Velocity targetVel = { .vx = 0.0f, .vy = 0.0f, .omega = 0.0f };

    switch(code) {
        case 0b0110:
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = 0;
            executePulse(targetVel, CONTROL_PERIOD);
            break;
        case 0b0100:
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;
        case 0b1000: case 0b1100: case 0b1110:
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;
        case 0b0010:
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;
        case 0b0001: case 0b0011: case 0b0111:
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;
        case 0b0000:
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, SEARCH_PERIOD);
            break;
        case 0b1111:
            if (!g_has_avoided) {
                targetVel.vy = (float)baseSpeed;
                executePulse(targetVel, CONTROL_PERIOD);
            } else {
                stopMotors();
                return true; 
            }
            break;
        default: 
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, CONTROL_PERIOD);
            break;
    }
    return false;
}

// ==================== 7. 系统任务 (测速/屏幕刷UI/主控) ====================

// 7.1 LVGL UI 定时刷新与驱动 Task
void guiTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char buf[32];

    while (1) {
        // 主 LVGL 核心逻辑高频刷新 (5ms) 保证流畅渲染
        lv_timer_handler();

        // 精确每 1000ms (1 秒) 强制刷新显示文本
        if (xTaskGetTickCount() - xLastWakeTime >= pdMS_TO_TICKS(1000)) {
            xLastWakeTime = xTaskGetTickCount();

            // 主动更新测速 RPM
            getAllWheelRPM();

            if (lbl_wheel_a && lbl_wheel_b && lbl_wheel_d && lbl_dist) {
                // 安全转为整数+小数点打印，防止 %f 支持失效
                snprintf(buf, sizeof(buf), "WheelA: %d.%d", (int)g_current_rpm.rpm_A, (int)fabs(g_current_rpm.rpm_A * 10) % 10);
                lv_label_set_text(lbl_wheel_a, buf);

                snprintf(buf, sizeof(buf), "WheelB: %d.%d", (int)g_current_rpm.rpm_B, (int)fabs(g_current_rpm.rpm_B * 10) % 10);
                lv_label_set_text(lbl_wheel_b, buf);

                snprintf(buf, sizeof(buf), "WheelD: %d.%d", (int)g_current_rpm.rpm_D, (int)fabs(g_current_rpm.rpm_D * 10) % 10);
                lv_label_set_text(lbl_wheel_d, buf);

                snprintf(buf, sizeof(buf), "US Dist: %d.%dcm", (int)g_current_distance, (int)fabs(g_current_distance * 10) % 10);
                lv_label_set_text(lbl_dist, buf);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 7.2 主流程控制任务
void mainControlTask(void* pvParameters) {
    motorInit();
    sensorInit();
    ultrasonicInit();
    encoderInit(); 
    
    ESP_LOGI(TAG, "🚀 阶段 1：循迹控制，监控障碍物...");

    while (1) {
        float distance = getUltrasonicDistance();
        if (distance > 0.0f && distance <= 5.0f) {
            ESP_LOGW(TAG, "🚨 障碍触发 (%.2f cm)！避障中...", distance);
            // stopMotors();
            // vTaskDelay(pdMS_TO_TICKS(300));
            break;
        }
        controlLoop(BASE_SPEED);
    }

    // 阶段 2：平移避障
    Velocity vel_strafe_away = { .vx = (float)STRAFE_SPEED, .vy = 0.0f, .omega = 0.0f };
    executePulse(vel_strafe_away, 800); 

    while (1) {
        executePulse(vel_strafe_away, 100); 
        float distance = getUltrasonicDistance();
        if (distance > 5.0f) {
            executePulse(vel_strafe_away, 11);
            stopMotors(); 
            vTaskDelay(pdMS_TO_TICKS(400));
            break;
        }
    }

    // 阶段 3：直行 1.5s
    Velocity vel_fwd = { .vx = 0.0f, .vy = (float)BASE_SPEED, .omega = 0.0f };
    executePulse(vel_fwd, 1530); 
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(200));

    // 阶段 4：反向平移找黑线
    Velocity vel_strafe_back = { .vx = -(float)STRAFE_SPEED*0.90, .vy = 0.0f, .omega = 0.0f };
    while (1) {
        executePulse(vel_strafe_back, 25);
        SensorData data;
        readSensors(&data);
        if (getSensorCode(&data) != 0b0000) {
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }
    }

    // 阶段 5：循迹至终点
    g_has_avoided = true; 
    while (1) {
        bool should_stop = controlLoop(BASE_SPEED);
        if (should_stop) {
            ESP_LOGI(TAG, "🏁 终点到达，停机！");
            break;
        }
    }

    stopMotors();
    vTaskDelete(NULL);
}

// ==================== 8. 主入口函数 ====================
void app_main(void) {
    // 1. 初始化屏幕与 LVGL UI 界面
    lcdInitAndUiSetup();

    // 2. 启动屏幕 UI 刷新任务
    xTaskCreatePinnedToCore(guiTask, "gui_task", 4096, NULL, 2, NULL, 1);

    // 3. 启动小车逻辑与运动控制任务
    xTaskCreate(mainControlTask, "main_control_task", 4096, NULL, 5, NULL);
}