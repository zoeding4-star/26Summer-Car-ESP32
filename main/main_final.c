#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

// LVGL & LCD 依赖
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"

static const char* TAG = "ROBOT_SYSTEM";

// ==================== 1. 引脚定义 ====================
// 四路红外传感器 (0: 踩黑线, 1: 白地)
#define SENSOR_L2_PIN   GPIO_NUM_4
#define SENSOR_L1_PIN   GPIO_NUM_5
#define SENSOR_R1_PIN   GPIO_NUM_6
#define SENSOR_R2_PIN   GPIO_NUM_7

// 电机引脚
#define MOTOR_D_PWM     GPIO_NUM_14
#define MOTOR_D_IN1     GPIO_NUM_13
#define MOTOR_D_IN2     GPIO_NUM_12

#define MOTOR_A_PWM     GPIO_NUM_21
#define MOTOR_A_IN1     GPIO_NUM_46
#define MOTOR_A_IN2     GPIO_NUM_3

#define MOTOR_B_PWM     GPIO_NUM_15
#define MOTOR_B_IN1     GPIO_NUM_16
#define MOTOR_B_IN2     GPIO_NUM_17

// 超声波引脚
#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000     // 30ms 超时

// 霍尔编码器引脚
#define HALL_D_A_PIN    GPIO_NUM_10
#define HALL_D_B_PIN    GPIO_NUM_11
#define HALL_A_A_PIN    GPIO_NUM_20
#define HALL_A_B_PIN    GPIO_NUM_19
#define HALL_B_A_PIN    GPIO_NUM_42
#define HALL_B_B_PIN    GPIO_NUM_41

// LCD 屏幕引脚
#define PIN_NUM_CS      GPIO_NUM_2
#define PIN_NUM_SCK     GPIO_NUM_1
#define PIN_NUM_SDI     GPIO_NUM_38
#define PIN_NUM_DC      GPIO_NUM_39
#define PIN_NUM_RST     GPIO_NUM_40

// 屏幕分辨率 (ST7735 128x160)
#define LCD_H_RES       128
#define LCD_V_RES       160

// ==================== 2. 参数定义 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        1000
#define PWM_RESOL       8
#define MAX_SPEED       255

#define BASE_SPEED      80
#define STRAFE_SPEED    40
#define CONTROL_PERIOD  5
#define SEARCH_PERIOD   10

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f

#define MICRO_PULSE_MS  2
#define MACRO_PULSE_MS  3
#define OMEGA_MICRO     12.0f
#define OMEGA_MACRO     15.0f
#define OMEGA_SEARCH    300.0f

#define STRAFE_SCALE_D   0.90f     
#define STRAFE_SCALE_A   0.90f     
#define STRAFE_SCALE_B   1.15f     

#define FWD_SCALE_D      1.00f
#define FWD_SCALE_A      1.00f
#define FWD_SCALE_B      1.00f

#define ROT_SCALE_D      1.00f
#define ROT_SCALE_A      1.00f
#define ROT_SCALE_B      1.00f

#define HALL_LINES_PER_REV  512
#define HALL_QUADRATURE      4
#define HALL_PULSE_PER_REV   (HALL_LINES_PER_REV * HALL_QUADRATURE)

// ==================== 3. 结构体定义 ====================
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

typedef struct {
    volatile int32_t pulse_count;
    volatile int32_t last_pulse_count;
    volatile float speed_rpm;
    volatile int32_t direction;
    uint32_t last_update_time;
    uint32_t pulse_time_window;
    volatile uint8_t last_ab_state;
} HallSensorData;

// ==================== 4. 全局共享数据变量 ====================
static LastDirection g_last_dir = LAST_DIR_LEFT;
static bool g_has_avoided = false; 

// 共享的实时数据 (供 LCD 显示)
static HallSensorData hall_D = {0};
static HallSensorData hall_A = {0};
static HallSensorData hall_B = {0};
static volatile float g_current_distance = 0.0f;

// LVGL UI 标签句柄
static lv_obj_t *lbl_wheel_a = NULL;
static lv_obj_t *lbl_wheel_b = NULL;
static lv_obj_t *lbl_wheel_d = NULL;
static lv_obj_t *lbl_dist = NULL;

// LVGL 缓冲区
static lv_disp_draw_buf_t disp_buf;
static lv_color_t buf1[LCD_H_RES * 20];
static lv_color_t buf2[LCD_H_RES * 20];

// ==================== 5. 电机与底盘驱动 ====================
void motorInit(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_D_PWM) | (1ULL << MOTOR_D_IN1) | (1ULL << MOTOR_D_IN2) |
                        (1ULL << MOTOR_A_PWM) | (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_PWM) | (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
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
        .intr_type = LEDC_INTR_DISABLE,
        .duty = 0,
        .hpoint = 0
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

// ==================== 6. 红外传感器驱动 ====================
void sensorInit(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << SENSOR_L2_PIN) | (1ULL << SENSOR_L1_PIN) |
                        (1ULL << SENSOR_R1_PIN) | (1ULL << SENSOR_R2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
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

// ==================== 7. 超声波传感器驱动 ====================
void ultrasonicInit(void) {
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
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
    g_current_distance = dist; // 实时写全局变量
    return dist;
}

// ==================== 8. 霍尔编码器逻辑 ====================
static inline uint8_t getABState(gpio_num_t pin_a, gpio_num_t pin_b) {
    uint8_t state = 0;
    if (gpio_get_level(pin_a)) state |= 0x01;
    if (gpio_get_level(pin_b)) state |= 0x02;
    return state;
}

static inline void decodeAB(gpio_num_t pin_a, gpio_num_t pin_b, HallSensorData* hall) {
    uint8_t current_state = getABState(pin_a, pin_b);
    uint8_t last_state = hall->last_ab_state;
    
    if (current_state != last_state) {
        switch (last_state) {
            case 0:
                if (current_state == 1) { hall->pulse_count++; hall->direction = 1; }
                else if (current_state == 2) { hall->pulse_count--; hall->direction = -1; }
                break;
            case 1:
                if (current_state == 3) { hall->pulse_count++; hall->direction = 1; }
                else if (current_state == 0) { hall->pulse_count--; hall->direction = -1; }
                break;
            case 2:
                if (current_state == 0) { hall->pulse_count++; hall->direction = 1; }
                else if (current_state == 3) { hall->pulse_count--; hall->direction = -1; }
                break;
            case 3:
                if (current_state == 2) { hall->pulse_count++; hall->direction = 1; }
                else if (current_state == 1) { hall->pulse_count--; hall->direction = -1; }
                break;
        }
        hall->last_ab_state = current_state;
    }
}

static void IRAM_ATTR hall_isr_handler(void* arg) {
    decodeAB(HALL_D_A_PIN, HALL_D_B_PIN, &hall_D);
    decodeAB(HALL_A_A_PIN, HALL_A_B_PIN, &hall_A);
    decodeAB(HALL_B_A_PIN, HALL_B_B_PIN, &hall_B);
}

void hallInit(void) {
    uint64_t hall_pin_mask = (1ULL << HALL_D_A_PIN) | (1ULL << HALL_D_B_PIN) |
                             (1ULL << HALL_A_A_PIN) | (1ULL << HALL_A_B_PIN) |
                             (1ULL << HALL_B_A_PIN) | (1ULL << HALL_B_B_PIN);
    
    gpio_config_t hall_conf = {
        .pin_bit_mask = hall_pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    gpio_config(&hall_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(HALL_D_A_PIN, hall_isr_handler, NULL);
    gpio_isr_handler_add(HALL_D_B_PIN, hall_isr_handler, NULL);
    gpio_isr_handler_add(HALL_A_A_PIN, hall_isr_handler, NULL);
    gpio_isr_handler_add(HALL_A_B_PIN, hall_isr_handler, NULL);
    gpio_isr_handler_add(HALL_B_A_PIN, hall_isr_handler, NULL);
    gpio_isr_handler_add(HALL_B_B_PIN, hall_isr_handler, NULL);

    uint32_t current_time = esp_timer_get_time() / 1000;
    hall_D.last_update_time = current_time;
    hall_A.last_update_time = current_time;
    hall_B.last_update_time = current_time;
    hall_D.pulse_time_window = 50;
    hall_A.pulse_time_window = 50;
    hall_B.pulse_time_window = 50;
    hall_D.last_ab_state = getABState(HALL_D_A_PIN, HALL_D_B_PIN);
    hall_A.last_ab_state = getABState(HALL_A_A_PIN, HALL_A_B_PIN);
    hall_B.last_ab_state = getABState(HALL_B_A_PIN, HALL_B_B_PIN);
}

void calculateWheelSpeed(HallSensorData* hall) {
    uint32_t current_time = esp_timer_get_time() / 1000;
    if (current_time - hall->last_update_time >= hall->pulse_time_window) {
        float delta_time = (float)(current_time - hall->last_update_time) / 1000.0f;
        int32_t delta_pulses = hall->pulse_count - hall->last_pulse_count;
        hall->speed_rpm = (float)delta_pulses / (HALL_PULSE_PER_REV * delta_time) * 60.0f;
        hall->last_pulse_count = hall->pulse_count;
        hall->last_update_time = current_time;
    }
}

// ==================== 9. 运动学逆解 & 循迹控制逻辑 ====================
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
        wheels.D = raw_D;
        wheels.A = raw_A;
        wheels.B = raw_B;
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
            targetVel.vx = 0.0f;
            targetVel.vy = 0.0f; 
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, SEARCH_PERIOD);
            break;

        case 0b1111:
            if (!g_has_avoided) {
                targetVel.vy = (float)baseSpeed;
                targetVel.omega = 0;
                executePulse(targetVel, CONTROL_PERIOD);
            } else {
                stopMotors();
                return true; 
            }
            break;

        default: 
            targetVel.vx = 0.0f;
            targetVel.vy = 0.0f;
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, CONTROL_PERIOD);
            break;
    }
    return false;
}

// ==================== 10. LVGL 屏幕驱动与 UI 任务 ====================
static void increase_lvgl_tick(void *arg) {
    lv_tick_inc(2);
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    esp_lcd_panel_draw_bitmap(panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

void lcdInit(esp_lcd_panel_handle_t *out_panel) {
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

    *out_panel = panel_handle;
}

void guiTask(void *pvParameters) {
    esp_lcd_panel_handle_t panel_handle = NULL;
    lcdInit(&panel_handle);

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

    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2000));

    // UI 构建 (白底黑字)
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

    lbl_wheel_b = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_b, lv_color_black(), LV_PART_MAIN);

    lbl_wheel_d = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_wheel_d, lv_color_black(), LV_PART_MAIN);

    lbl_dist = lv_label_create(cont);
    lv_obj_set_style_text_color(lbl_dist, lv_color_black(), LV_PART_MAIN);

    while (1) {
        // 1. 计算三个车轮当前转速
        calculateWheelSpeed(&hall_A);
        calculateWheelSpeed(&hall_B);
        calculateWheelSpeed(&hall_D);

        // 2. 刷新动态显示数据
        lv_label_set_text_fmt(lbl_wheel_a, "WheelA: %.1f", hall_A.speed_rpm);
        lv_label_set_text_fmt(lbl_wheel_b, "WheelB: %.1f", hall_B.speed_rpm);
        lv_label_set_text_fmt(lbl_wheel_d, "WheelD: %.1f", hall_D.speed_rpm);
        
        if (g_current_distance < 0) {
            lv_label_set_text_fmt(lbl_dist, "US Dist: --");
        } else {
            lv_label_set_text_fmt(lbl_dist, "US Dist: %.1fcm", g_current_distance);
        }

        lv_timer_handler();
        vTaskDelay(pdMS_TO_TICKS(30)); // 屏幕以大约 33FPS 帧率刷新
    }
}

// ==================== 11. 主运动控制任务 ====================
void mainControlTask(void* pvParameters) {
    motorInit();
    sensorInit();
    ultrasonicInit();
    hallInit(); // 启动霍尔测速

    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "🚀 阶段 1：开始初始循迹，监测 10cm 避障障碍物...");

    // ------------------- 阶段 1：循迹 + 超声波检测 -------------------
    while (1) {
        float distance = getUltrasonicDistance();

        if (distance > 0.0f && distance <= 10.0f) {
            ESP_LOGW(TAG, "🚨 前方障碍 (%.2f cm <= 10cm)！准备避障...", distance);
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(300));
            break;
        }

        controlLoop(BASE_SPEED);
    }

    // ------------------- 阶段 2：平移避障逻辑 -------------------
    Velocity vel_strafe_away = { .vx = (float)STRAFE_SPEED, .vy = 0.0f, .omega = 0.0f };
    
    ESP_LOGI(TAG, "➡️ 阶段 2-1：强行平移 1 秒，确保偏离原轨道...");
    executePulse(vel_strafe_away, 1000); 

    ESP_LOGI(TAG, "➡️ 阶段 2-2：持续平移，等待前方距离清空 (>20cm)...");
    while (1) {
        executePulse(vel_strafe_away, 50); 

        float distance = getUltrasonicDistance();

        if (distance > 20.0f) {
            ESP_LOGI(TAG, "✅ 前方障碍已清空 (%.2f cm > 20cm)，额外补走 15ms 余量...", distance);
            executePulse(vel_strafe_away, 15); 
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }
    }

    // ------------------- 阶段 3：直行 1.8 秒 -------------------
    ESP_LOGI(TAG, "⬆️ 阶段 3：绕过障碍，前进步进 1.8 秒...");
    Velocity vel_fwd = { .vx = 0.0f, .vy = (float)BASE_SPEED, .omega = 0.0f };
    executePulse(vel_fwd, 1800); 
    stopMotors();
    vTaskDelay(pdMS_TO_TICKS(200));

    // ------------------- 阶段 4：反向平移寻找黑线 -------------------
    ESP_LOGI(TAG, "⬅️ 阶段 4：反向平移，寻找黑线...");
    Velocity vel_strafe_back = { .vx = -(float)STRAFE_SPEED, .vy = 0.0f, .omega = 0.0f };
    while (1) {
        executePulse(vel_strafe_back, 30);

        SensorData data;
        readSensors(&data);
        int code = getSensorCode(&data);

        if (code != 0b0000) {
            ESP_LOGI(TAG, "🎯 重新捕捉到黑线 (Sensor code: 0x%X)！停止平移。", code);
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }
    }

    // ------------------- 阶段 5：二次循迹直至全黑 -------------------
    g_has_avoided = true; 
    ESP_LOGI(TAG, "🚗 阶段 5：恢复沿黑线循迹，遇到全黑立即停止...");

    while (1) {
        bool should_stop = controlLoop(BASE_SPEED);
        if (should_stop) {
            ESP_LOGI(TAG, "🏁 避障后检测到全黑 (0b1111)，任务终点到达，安全停机！");
            break;
        }
    }

    stopMotors();
    vTaskDelete(NULL);
}

// ==================== 12. 系统总入口 ====================
void app_main(void) {
    // 启动屏幕显示任务 (优先级为 3)
    xTaskCreate(guiTask, "gui_task", 4096, NULL, 3, NULL);

    // 启动核心控制任务 (高优先级 5，保证实时性)
    xTaskCreate(mainControlTask, "main_control_task", 4096, NULL, 5, NULL);
}