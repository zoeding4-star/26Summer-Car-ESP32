#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "FULL_TASK";

// ==================== 1. 引脚定义 ====================
// 四路红外传感器 (0: 踩黑线, 1: 白地)
#define SENSOR_L2_PIN   GPIO_NUM_4
#define SENSOR_L1_PIN   GPIO_NUM_5
#define SENSOR_R1_PIN   GPIO_NUM_6
#define SENSOR_R2_PIN   GPIO_NUM_7

// 左前电机 (D)
#define MOTOR_D_PWM     GPIO_NUM_14
#define MOTOR_D_IN1     GPIO_NUM_13
#define MOTOR_D_IN2     GPIO_NUM_12

// 右前电机 (A)
#define MOTOR_A_PWM     GPIO_NUM_21
#define MOTOR_A_IN1     GPIO_NUM_46
#define MOTOR_A_IN2     GPIO_NUM_3

// 后轮电机 (B)
#define MOTOR_B_PWM     GPIO_NUM_15
#define MOTOR_B_IN1     GPIO_NUM_16
#define MOTOR_B_IN2     GPIO_NUM_17

// 超声波传感器
#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000     // 30ms 超时

// ==================== 2. PWM & 物理控制参数 ====================
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        1000
#define PWM_RESOL       8
#define MAX_SPEED       255

#define BASE_SPEED      80      // 循迹/直行基础速度
#define STRAFE_SPEED    45      // 横向平移基础速度
#define CONTROL_PERIOD  5       // 循迹脉冲周期 (ms)
#define SEARCH_PERIOD   10      // 脱线寻线周期 (ms)

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f

// 微调参数
#define MICRO_PULSE_MS  2
#define MACRO_PULSE_MS  3
#define OMEGA_MICRO     12.0f
#define OMEGA_MACRO     15.0f
#define OMEGA_SEARCH    300.0f

// 多模态轮速 Scale
#define STRAFE_SCALE_D   0.85f     
#define STRAFE_SCALE_A   0.85f     
#define STRAFE_SCALE_B   1.14f     

#define FWD_SCALE_D      1.00f
#define FWD_SCALE_A      1.00f
#define FWD_SCALE_B      1.00f

#define ROT_SCALE_D      1.00f
#define ROT_SCALE_A      1.00f
#define ROT_SCALE_B      1.00f

// ==================== 3. 全局数据结构 & 变量 ====================
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

// 避障状态 Flag (false: 未进行过避障; true: 避障完成后)
static bool g_has_avoided = false; 

// ==================== 4. 驱动与底层控制 ====================
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
    return (float)(echo_end - echo_start) * 0.0343f / 2.0f;
}

// ==================== 5. 运动学逆解 ====================
MotorSpeed inverseKinematics(const Velocity* vel) {
    MotorSpeed wheels;
    float L = WHEEL_DISTANCE;
    
    // 保持你原始的逆解定义
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

// ==================== 6. 循迹单步控制逻辑 ====================
bool controlLoop(int baseSpeed) {
    SensorData data;
    readSensors(&data);
    int code = getSensorCode(&data);

    Velocity targetVel = { .vx = 0.0f, .vy = 0.0f, .omega = 0.0f };

    switch(code) {
        case 0b0110: // 中间双灯：完美直行
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = 0;
            executePulse(targetVel, CONTROL_PERIOD);
            break;

        case 0b0100: // 仅 L1 亮
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;

        case 0b1000: case 0b1100: case 0b1110: // 偏右较多/弯道
            g_last_dir = LAST_DIR_LEFT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = -OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;

        case 0b0010: // 仅 R1 亮
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MICRO;
            executePulse(targetVel, MICRO_PULSE_MS);
            break;

        case 0b0001: case 0b0011: case 0b0111: // 偏左较多/弯道
            g_last_dir = LAST_DIR_RIGHT;
            targetVel.vy = (float)baseSpeed;
            targetVel.omega = OMEGA_MACRO;
            executePulse(targetVel, MACRO_PULSE_MS);
            break;

        case 0b0000: // 全白脱线
            targetVel.vx = 0.0f;
            targetVel.vy = 0.0f; 
            targetVel.omega = (g_last_dir == LAST_DIR_LEFT) ? -OMEGA_SEARCH : OMEGA_SEARCH;
            executePulse(targetVel, SEARCH_PERIOD);
            break;

        case 0b1111: // 四传感器全黑
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

// ==================== 主流程状态机控制任务 ====================
void mainControlTask(void* pvParameters) {
    motorInit();
    sensorInit();
    ultrasonicInit();
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG, "🚀 阶段 1：开始初始循迹，监测 10cm 避障障碍物...");

    // ------------------- 阶段 1：循迹 + 超声波检测 -------------------
    while (1) {
        float distance = getUltrasonicDistance();

        // 过滤负数异常码，仅在有效距离 (0, 10cm] 内触发避障
        if (distance > 0.0f && distance <= 10.0f) {
            ESP_LOGW(TAG, "🚨 前方障碍 (%.2f cm <= 10cm)！准备避障...", distance);
            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(300));
            break;
        }

        controlLoop(BASE_SPEED);
    }

    // ------------------- 阶段 2：完全照搬测试代码的平移避障逻辑 -------------------
    Velocity vel_strafe_away = { .vx = (float)STRAFE_SPEED, .vy = 0.0f, .omega = 0.0f };
    
    ESP_LOGI(TAG, "➡️ 阶段 2-1：强行平移 1 秒，确保偏离原轨道...");
    executePulse(vel_strafe_away, 1000); 

    ESP_LOGI(TAG, "➡️ 阶段 2-2：持续平移，等待前方距离清空 (>20cm)...");
    while (1) {
        executePulse(vel_strafe_away, 50); 

        float distance = getUltrasonicDistance();

        // 纯粹按测试代码条件：仅当测量到真实有效距离且大于 20cm 时退出
        if (distance > 20.0f) {
            ESP_LOGI(TAG, "✅ 前方障碍已清空 (%.2f cm > 20cm)，额外补走 10ms 余量...", distance);
            
            // 额外多走 10ms 平移，确保彻底避开边界
            executePulse(vel_strafe_away, 10); 

            stopMotors();
            vTaskDelay(pdMS_TO_TICKS(200));
            break;
        }
    }

    // ------------------- 阶段 3：直行 2.5 秒 -------------------
    ESP_LOGI(TAG, "⬆️ 阶段 3：绕过障碍，前进步进 2.5 秒...");
    Velocity vel_fwd = { .vx = 0.0f, .vy = (float)BASE_SPEED, .omega = 0.0f };
    executePulse(vel_fwd, 2500); 
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

void app_main(void) {
    xTaskCreate(mainControlTask, "main_control_task", 4096, NULL, 5, NULL);
}