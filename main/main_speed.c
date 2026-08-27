#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "HALL_TEST";

// ==================== 霍尔传感器引脚（512线AB相） ====================
#define HALL_D_A_PIN    GPIO_NUM_10      // 左前轮霍尔A相
#define HALL_D_B_PIN    GPIO_NUM_11      // 左前轮霍尔B相
#define HALL_A_A_PIN    GPIO_NUM_20      // 右前轮霍尔A相
#define HALL_A_B_PIN    GPIO_NUM_19      // 右前轮霍尔B相
#define HALL_B_A_PIN    GPIO_NUM_42      // 后轮霍尔A相
#define HALL_B_B_PIN    GPIO_NUM_41      // 后轮霍尔B相

// ==================== 霍尔传感器参数 ====================
#define HALL_LINES_PER_REV  512
#define HALL_QUADRATURE      4
#define HALL_PULSE_PER_REV   (HALL_LINES_PER_REV * HALL_QUADRATURE)

// ==================== 霍尔数据结构 ====================
typedef struct {
    volatile int32_t pulse_count;
    volatile int32_t last_pulse_count;
    volatile float speed_rpm;
    volatile int32_t direction;
    uint32_t last_update_time;
    uint32_t pulse_time_window;
    volatile uint8_t last_ab_state;
} HallSensorData;

// ==================== 全局变量 ====================
static HallSensorData hall_D = {0};
static HallSensorData hall_A = {0};
static HallSensorData hall_B = {0};

// ==================== AB相解码函数 ====================
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

// ==================== 中断服务函数 ====================
static void IRAM_ATTR hall_isr_handler(void* arg) {
    decodeAB(HALL_D_A_PIN, HALL_D_B_PIN, &hall_D);
    decodeAB(HALL_A_A_PIN, HALL_A_B_PIN, &hall_A);
    decodeAB(HALL_B_A_PIN, HALL_B_B_PIN, &hall_B);
}

// ==================== 霍尔初始化 ====================
void hallInit(void) {
    ESP_LOGI(TAG, "🔄 初始化霍尔传感器...");
    
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

    ESP_LOGI(TAG, "✅ 霍尔初始化完成！");
    ESP_LOGI(TAG, "📊 编码器参数: %d线, 4倍频 = %d 脉冲/转", HALL_LINES_PER_REV, HALL_PULSE_PER_REV);
}

// ==================== 计算速度 ====================
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

// ==================== 打印霍尔状态 ====================
void printHallStatus(void) {
    // 计算速度
    calculateWheelSpeed(&hall_D);
    calculateWheelSpeed(&hall_A);
    calculateWheelSpeed(&hall_B);
    
    // 获取AB状态
    uint8_t state_D = getABState(HALL_D_A_PIN, HALL_D_B_PIN);
    uint8_t state_A = getABState(HALL_A_A_PIN, HALL_A_B_PIN);
    uint8_t state_B = getABState(HALL_B_A_PIN, HALL_B_B_PIN);
    
    // 打印
    printf("\n");
    printf("═══════════════════════════════════════════════\n");
    printf("  📊 霍尔传感器实时数据\n");
    printf("═══════════════════════════════════════════════\n");
    
    printf(" 左前轮(D) | RPM:%6.0f | 方向:%s | 脉冲:%6ld | AB:%d%d\n",
             hall_D.speed_rpm,
             hall_D.direction == 1 ? "正转▶" : (hall_D.direction == -1 ? "反转◀" : "停止■"),
             (long)hall_D.pulse_count,
             (state_D & 0x01) ? 1 : 0,
             (state_D & 0x02) ? 1 : 0);
    
    printf(" 右前轮(A) | RPM:%6.0f | 方向:%s | 脉冲:%6ld | AB:%d%d\n",
             hall_A.speed_rpm,
             hall_A.direction == 1 ? "正转▶" : (hall_A.direction == -1 ? "反转◀" : "停止■"),
             (long)hall_A.pulse_count,
             (state_A & 0x01) ? 1 : 0,
             (state_A & 0x02) ? 1 : 0);
    
    printf(" 后轮  (B) | RPM:%6.0f | 方向:%s | 脉冲:%6ld | AB:%d%d\n",
             hall_B.speed_rpm,
             hall_B.direction == 1 ? "正转▶" : (hall_B.direction == -1 ? "反转◀" : "停止■"),
             (long)hall_B.pulse_count,
             (state_B & 0x01) ? 1 : 0,
             (state_B & 0x02) ? 1 : 0);
    
    printf("═══════════════════════════════════════════════\n");
    printf("  💡 手动转动车轮观察数据变化\n");
    printf("═══════════════════════════════════════════════\n\n");
}

// ==================== 主任务 ====================
void hallTestTask(void* pvParameters) {
    printf("\n");
    printf("═══════════════════════════════════════════════\n");
    printf("  🔍 霍尔传感器独立测试程序\n");
    printf("  📌 请手动转动车轮，观察数据变化\n");
    printf("═══════════════════════════════════════════════\n\n");
    
    // 初始化霍尔
    hallInit();
    
    vTaskDelay(pdMS_TO_TICKS(500));
    
    printf("\n");
    printf("  ✅ 霍尔初始化成功！开始监测...\n");
    printf("  📌 编码器: 512线, 4倍频 = %d 脉冲/转\n\n", HALL_PULSE_PER_REV);
    
    // 每秒打印一次数据
    while (1) {
        printHallStatus();
        vTaskDelay(pdMS_TO_TICKS(500));  // 每500ms更新一次
    }
}

void app_main(void) {
    xTaskCreate(hallTestTask, "hall_test", 4096, NULL, 5, NULL);
}