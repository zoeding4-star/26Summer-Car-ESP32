#include "encoder.h"
#include "ir_config.h"

#include "esp_err.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"

static IrEncoderHandle g_encoders[3] = {0};
static IrAllWheelRPM g_current_rpm = {0};

static void init_single_encoder(gpio_num_t pin_a, gpio_num_t pin_b, IrEncoderHandle *handle)
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

    handle->pulse_count = 0;
    handle->last_count = 0;
}

void ir_encoder_init(void)
{
    init_single_encoder(IR_HALL_D_A_PIN, IR_HALL_D_B_PIN, &g_encoders[IR_WHEEL_D]);
    init_single_encoder(IR_HALL_A_A_PIN, IR_HALL_A_B_PIN, &g_encoders[IR_WHEEL_A]);
    init_single_encoder(IR_HALL_B_A_PIN, IR_HALL_B_B_PIN, &g_encoders[IR_WHEEL_B]);
}

IrAllWheelRPM ir_get_all_wheel_rpm(void)
{
    IrAllWheelRPM result = {0.0f, 0.0f, 0.0f};

    static int64_t last_time = 0;
    static int32_t last_count_D = 0, last_count_A = 0, last_count_B = 0;

    int64_t now = esp_timer_get_time();

    if (last_time == 0) {
        pcnt_unit_get_count(g_encoders[IR_WHEEL_D].unit, (int *)&last_count_D);
        pcnt_unit_get_count(g_encoders[IR_WHEEL_A].unit, (int *)&last_count_A);
        pcnt_unit_get_count(g_encoders[IR_WHEEL_B].unit, (int *)&last_count_B);
        last_time = now;
        return result;
    }

    int64_t elapsed_us = now - last_time;
    if (elapsed_us < IR_SAMPLE_TIME_MS * 1000) return g_current_rpm;

    int count_D = 0, count_A = 0, count_B = 0;
    pcnt_unit_get_count(g_encoders[IR_WHEEL_D].unit, &count_D);
    pcnt_unit_get_count(g_encoders[IR_WHEEL_A].unit, &count_A);
    pcnt_unit_get_count(g_encoders[IR_WHEEL_B].unit, &count_B);

    int32_t delta_D = count_D - last_count_D;
    int32_t delta_A = count_A - last_count_A;
    int32_t delta_B = count_B - last_count_B;

    last_count_D = count_D; last_count_A = count_A; last_count_B = count_B;

    float minutes = (float)elapsed_us / 60000000.0f;
    result.rpm_D = ((float)delta_D / IR_ENCODER_PPR) / minutes;
    result.rpm_A = ((float)delta_A / IR_ENCODER_PPR) / minutes;
    result.rpm_B = ((float)delta_B / IR_ENCODER_PPR) / minutes;

    last_time = now;
    g_current_rpm = result;
    return result;
}

IrAllWheelRPM ir_telemetry_rpm(void)
{
    return g_current_rpm;
}
