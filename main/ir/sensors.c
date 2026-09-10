#include "sensors.h"
#include "ir_config.h"

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

static float g_current_distance = 0.0f;

void ir_sensor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_SENSOR_L2_PIN) | (1ULL << IR_SENSOR_L1_PIN) |
                        (1ULL << IR_SENSOR_R1_PIN) | (1ULL << IR_SENSOR_R2_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&io_conf);
}

void ir_read_sensors(IrSensorData *data)
{
    data->l2 = gpio_get_level(IR_SENSOR_L2_PIN);
    data->l1 = gpio_get_level(IR_SENSOR_L1_PIN);
    data->r1 = gpio_get_level(IR_SENSOR_R1_PIN);
    data->r2 = gpio_get_level(IR_SENSOR_R2_PIN);
}

int ir_get_sensor_code(const IrSensorData *data)
{
    int s_l2 = (data->l2 == 0) ? 1 : 0;
    int s_l1 = (data->l1 == 0) ? 1 : 0;
    int s_r1 = (data->r1 == 0) ? 1 : 0;
    int s_r2 = (data->r2 == 0) ? 1 : 0;
    return (s_l2 << 3) | (s_l1 << 2) | (s_r1 << 1) | s_r2;
}

void ir_ultrasonic_init(void)
{
    gpio_config_t trig_conf = {
        .pin_bit_mask = (1ULL << IR_TRIG_PIN),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trig_conf);

    gpio_config_t echo_conf = {
        .pin_bit_mask = (1ULL << IR_ECHO_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    gpio_config(&echo_conf);

    gpio_set_level(IR_TRIG_PIN, 0);
}

float ir_get_ultrasonic_distance(void)
{
    gpio_set_level(IR_TRIG_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(IR_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(IR_TRIG_PIN, 0);

    int64_t start_wait = esp_timer_get_time();
    while (gpio_get_level(IR_ECHO_PIN) == 0) {
        if (esp_timer_get_time() - start_wait > IR_TIMEOUT_US) return -1.0f;
    }

    int64_t echo_start = esp_timer_get_time();
    while (gpio_get_level(IR_ECHO_PIN) == 1) {
        if (esp_timer_get_time() - echo_start > IR_TIMEOUT_US) return -2.0f;
    }

    int64_t echo_end = esp_timer_get_time();
    float dist = (float)(echo_end - echo_start) * 0.0343f / 2.0f;

    if (dist > 0) g_current_distance = dist;
    return dist;
}

float ir_telemetry_distance(void)
{
    return g_current_distance;
}
