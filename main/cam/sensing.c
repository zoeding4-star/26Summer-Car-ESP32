#include "sensing.h"
#include "cam_config.h"
#include "cam_state.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

static EncoderHandle_t g_encoders[3];

void ultrasonic_init(void)
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

float ultrasonic_cm(void)
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

void encoder_init(void)
{
    init_single_encoder(HALL_D_A_PIN, HALL_D_B_PIN, &g_encoders[WHEEL_D]);
    init_single_encoder(HALL_A_A_PIN, HALL_A_B_PIN, &g_encoders[WHEEL_A]);
    init_single_encoder(HALL_B_A_PIN, HALL_B_B_PIN, &g_encoders[WHEEL_B]);
}

AllWheelRPM get_all_wheel_rpm(void)
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
