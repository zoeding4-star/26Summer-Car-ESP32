// ==================== 修正后的完整循迹代码 ====================
// 已移除死区补偿，整体速度与微调幅度均减半

#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

static const char* TAG = "LINE_FOLLOWER";

// ==================== 引脚定义 ====================
#define SENSOR_L2_PIN   GPIO_NUM_4
#define SENSOR_L1_PIN   GPIO_NUM_5
#define SENSOR_R1_PIN   GPIO_NUM_6
#define SENSOR_R2_PIN   GPIO_NUM_7

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

// ==================== PWM配置 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_RESOL       8
#define PWM_FREQ        1000
#define MAX_SPEED       255

// ==================== 控制参数 ====================
#define BASE_SPEED      80      // 基础直行速度（减半）
#define CONTROL_PERIOD  5      // 控制周期 (ms)
#define SEARCH_PERIOD   10      // 脱线自转周期 (ms)

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f

// 【微调参数：速度与脉冲时长均大幅减小】
#define MICRO_PULSE_MS  2       // 内侧灯微调时长 (2ms)
#define MACRO_PULSE_MS  3       // 外侧灯微调时长 (3ms)
#define OMEGA_MICRO     15.0f   // 轻微调整角速度（减半）
#define OMEGA_MACRO     20.0f   // 稍大调整角速度（减半）
#define OMEGA_SEARCH    300.0f   // 全白脱线自转角速度（减半）

// ==================== 数据结构 ====================
typedef struct {
    int l2, l1, r1, r2;
} SensorData;

typedef struct {
    float D;  // 左前
    float A;  // 右前
    float B;  // 后轮
} MotorSpeed;

typedef struct {
    float vx, vy, omega;
} Velocity;

// 上一次记忆的偏离方向
typedef enum {
    LAST_DIR_LEFT,
    LAST_DIR_RIGHT
} LastDirection;

static LastDirection g_last_dir = LAST_DIR_LEFT;

// ==================== 电机初始化 ====================
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
    
    ESP_LOGI(TAG, "✅ 电机初始化完成");
}

// ==================== 设置单个电机（已移除死区补偿） ====================
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

// ==================== 设置所有电机 ====================
void setAllMotors(const MotorSpeed* speed) {
    setMotor(MOTOR_D_PWM, MOTOR_D_IN1, MOTOR_D_IN2, speed->D, LEDC_CH_D);
    setMotor(MOTOR_A_PWM, MOTOR_A_IN1, MOTOR_A_IN2, speed->A, LEDC_CH_A);
    setMotor(MOTOR_B_PWM, MOTOR_B_IN1, MOTOR_B_IN2, speed->B, LEDC_CH_B);
}

void stopMotors(void) {
    MotorSpeed zero = {0, 0, 0};
    setAllMotors(&zero);
}

// ==================== 传感器读取 ====================
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

// ==================== 运动学逆解 ====================
MotorSpeed inverseKinematics(const Velocity* vel) {
    MotorSpeed wheels;
    float L = WHEEL_DISTANCE;
    
    wheels.D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    wheels.A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    wheels.B = -vel->vx + L * vel->omega;
    
    return wheels;
}

// 执行指定的速度并保持固定时间
void executePulse(Velocity vel, int duration_ms) {
    MotorSpeed speed = inverseKinematics(&vel);
    setAllMotors(&speed);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
}

// ==================== 修正后的 controlLoop ====================
void controlLoop(int baseSpeed) {
    SensorData data;
    readSensors(&data);
    int code = getSensorCode(&data);

    // 1. 默认初始速度清零，防止任何分支意外带入直行分量
    Velocity targetVel = { .vx = 0.0f, .vy = 0.0f, .omega = 0.0f };

    switch(code) {
        case 0b0110: // 中间双灯：完美直行
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = 0;
            executePulse(targetVel, CONTROL_PERIOD);
            break;

        // ---------- 左偏纠偏（向右转） ----------
        case 0b0100: // 仅 L1 亮：轻微偏右
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;

        case 0b1000: // 仅 L2 亮：偏右较多
        case 0b1100: // L1+L2 亮：压在左侧大弯上
        case 0b1110: // L1+L2+R1 亮：压在左侧大弯上
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;

        // ---------- 右偏纠偏（向左转） ----------
        case 0b0010: // 仅 R1 亮：轻微偏左
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;

        case 0b0001: // 仅 R2 亮：偏左较多
        case 0b0011: // R1+R2 亮：压在右侧大弯上
        case 0b0111: // L1+R1+R2 亮：压在右侧大弯上
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;

        // ---------- 完全脱线寻线（全白） ----------
        case 0b0000: // 全白脱线：强制 vy=0，纯自转
            targetVel.vx = 0.0f;
            targetVel.vy = 0.0f; 
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, SEARCH_PERIOD);
            break;

        case 0b1111: // 全黑或十字路口：保持直行
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = 0;
            executePulse(targetVel, CONTROL_PERIOD);
            break;

        default: 
            // 异常或未匹配状态：同样按脱线处理，原地自转寻线，绝不盲目向前冲
            targetVel.vx = 0.0f;
            targetVel.vy = 0.0f;
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, CONTROL_PERIOD);
            break;
    }
}

// ==================== 循迹任务 ====================
void lineFollowerTask(void* pvParameters) {
    int baseSpeed = BASE_SPEED;
    
    sensorInit();
    motorInit();
    
    ESP_LOGI(TAG, "🚗 循迹启动，基础速度=%d", baseSpeed);
    
    while(1) {
        controlLoop(baseSpeed);
    }
}

// ==================== ESP32入口 ====================
void app_main(void) {
    xTaskCreate(lineFollowerTask, "line_follower", 4096, NULL, 5, NULL);
}