#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/Task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "OMNI_CONTROL";

// ==================== 1. 电机引脚定义 (原始引脚) ====================
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

// ==================== 2. 超声波引脚与参数 ====================
#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000     // 30000us 超时限制

// ==================== 3. PWM与物理参数配置 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        1000
#define PWM_RESOL       8
#define MAX_SPEED       255
#define BASE_SPEED      55    // 横向平移基础速度
#define FORWARD_SPEED   80    // 直行基础速度

#define WHEEL_DISTANCE  0.1f       // 轮子到中心的距离 L
#define SIN_60          0.8660254f // sin(60°)
#define COS_60          0.5f       // cos(60°)

// ==================== 多模态独立轮速 Scale 定义 ====================
// 1. 横向平移专用 Scale (仅在 pure vx 时生效)
#define STRAFE_SCALE_D   0.85f     // 左前轮 (D)
#define STRAFE_SCALE_A   0.85f     // 右前轮 (A)
#define STRAFE_SCALE_B   1.15f     // 后轮 (B)

// 2. 纯直行/前后移动专用 Scale (循迹、直行使用)
#define FWD_SCALE_D      1.00f
#define FWD_SCALE_A      1.00f
#define FWD_SCALE_B      1.00f

// 3. 原地旋转专用 Scale (后续扩展旋转动作使用)
#define ROT_SCALE_D      1.00f
#define ROT_SCALE_A      1.00f
#define ROT_SCALE_B      1.00f

typedef struct {
    float D;  // 左前
    float A;  // 右前
    float B;  // 后轮
} MotorSpeed;

typedef struct {
    float vx, vy, omega;
} Velocity;

// ==================== 4. 电机驱动初始化与底层控制 ====================
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
    
    ESP_LOGI(TAG, "✅ 电机硬件初始化完成");
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

// ==================== 5. 超声波驱动与测距逻辑 ====================
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
    ESP_LOGI(TAG, "📡 超声波硬件初始化完成");
}

float getUltrasonicDistance(void) {
    gpio_set_level(TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIG_PIN, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_wait > TIMEOUT_US) {
            return -1.0f;
        }
    }

    int64_t echo_start = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > TIMEOUT_US) {
            return -2.0f;
        }
    }

    int64_t echo_end = esp_timer_get_time();
    int64_t duration = echo_end - echo_start;

    float distance = (float)duration * 0.0343f / 2.0f;
    return distance;
}

// ==================== 6. 精确运动学逆解（按运动模态应用独立 Scale） ====================
MotorSpeed inverseKinematics(const Velocity* vel) {
    MotorSpeed wheels;
    float L = WHEEL_DISTANCE;
    
    // 1. 计算三轮基础理论速度
    float raw_D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    float raw_A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    float raw_B = -(-vel->vx + L * vel->omega); 

    // 判断动作触发阈值
    bool is_strafe = (fabsf(vel->vx) > 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_forward = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) > 0.001f) && (fabsf(vel->omega) < 0.001f);
    bool is_rotate  = (fabsf(vel->vx) < 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) > 0.001f);

    // 2. 根据精确的运动模式匹配独立的调试系数
    if (is_strafe) {
        // 纯横向平移模式
        wheels.D = raw_D * STRAFE_SCALE_D;
        wheels.A = raw_A * STRAFE_SCALE_A;
        wheels.B = raw_B * STRAFE_SCALE_B;
    } 
    else if (is_forward) {
        // 纯直行/前后方向移动模式
        wheels.D = raw_D * FWD_SCALE_D;
        wheels.A = raw_A * FWD_SCALE_A;
        wheels.B = raw_B * FWD_SCALE_B;
    } 
    else if (is_rotate) {
        // 纯原地旋转模式
        wheels.D = raw_D * ROT_SCALE_D;
        wheels.A = raw_A * ROT_SCALE_A;
        wheels.B = raw_B * ROT_SCALE_B;
    } 
    else {
        // 复合运动（例如循迹过程中的微调：既有直行 vy，又有转向微调 omega）
        // 复合运动下不进行额外缩放，保持 1.0 原汁输出
        wheels.D = raw_D;
        wheels.A = raw_A;
        wheels.B = raw_B;
    }
    
    return wheels;
}

// ==================== 7. 主控任务：直行 -> 遇到障碍右平移 -> 避障后停止 ====================
void mainControlTask(void* pvParameters) {
    motorInit();
    ultrasonicInit();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "🚀 开始运行：直行并监测 10cm 障碍物");

    // 阶段 1：向正前方直行，检测是否靠近障碍物 (<=10cm)
    while (1) {
        float distance = getUltrasonicDistance();

        if (distance > 0) {
            ESP_LOGI(TAG, "📏 直行中，当前距离: %.2f cm", distance);
        } else {
            ESP_LOGW(TAG, "⚠️ 超声波测距异常代码: %.0f", distance);
        }

        if (distance > 0.0f && distance <= 10.0f) {
            ESP_LOGW(TAG, "🚨 检测到障碍物 (%.2f cm <= 10cm)！准备开始右平移避障...", distance);
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(300)); 
            break; 
        }

        // 纯直行 (+Y)，命中 is_forward 分支，使用 FWD_SCALE
        Velocity vel_forward = { .vx = 0.0f, .vy = (float)FORWARD_SPEED, .omega = 0.0f };
        MotorSpeed speed_forward = inverseKinematics(&vel_forward);
        setAllMotors(&speed_forward);

        vTaskDelay(pdMS_TO_TICKS(60)); 
    }

    // 阶段 2：向右平移，直到超声波测距 > 15cm 时停机
    ESP_LOGI(TAG, "➡️ 开始向右平移 (+X)，等待前方距离清空 (>15cm)...");
    
    while (1) {
        // 纯平移 (+X)，精确命中 is_strafe 分支，独享 STRAFE_SCALE
        Velocity vel_right = { .vx = (float)BASE_SPEED, .vy = 0.0f, .omega = 0.0f };
        MotorSpeed speed_right = inverseKinematics(&vel_right);
        setAllMotors(&speed_right);

        vTaskDelay(pdMS_TO_TICKS(60)); 

        float distance = getUltrasonicDistance();

        if (distance > 0) {
            ESP_LOGI(TAG, "📏 平移中，前方距离: %.2f cm", distance);
        }

        if (distance > 15.0f || distance == -2.0f) {
            ESP_LOGI(TAG, "✅ 前方距离已安全 (%.2f cm > 15cm)，停止横移并保持静止！", distance);
            stopMotors();
            break; 
        }
    }

    // 阶段 3：停机退出
    stopMotors();
    ESP_LOGI(TAG, "🏁 任务结束，小车已安全停止。");
    vTaskDelete(NULL);
}

void app_main(void) {
    xTaskCreate(mainControlTask, "main_control_task", 4096, NULL, 5, NULL);
}