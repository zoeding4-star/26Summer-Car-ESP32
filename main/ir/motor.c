#include "motor.h"
#include "ir_config.h"

#include <math.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

static void set_motor(gpio_num_t pwm, gpio_num_t in1, gpio_num_t in2,
                      float speed, ledc_channel_t channel)
{
    int pwm_val = (int)round(speed);
    if (pwm_val > IR_MAX_SPEED) pwm_val = IR_MAX_SPEED;
    if (pwm_val < -IR_MAX_SPEED) pwm_val = -IR_MAX_SPEED;

    if (pwm_val > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(IR_LEDC_MODE, channel, pwm_val);
    } else if (pwm_val < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        ledc_set_duty(IR_LEDC_MODE, channel, -pwm_val);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(IR_LEDC_MODE, channel, 0);
    }
    ledc_update_duty(IR_LEDC_MODE, channel);
}

void ir_motor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_MOTOR_D_PWM) | (1ULL << IR_MOTOR_D_IN1) | (1ULL << IR_MOTOR_D_IN2) |
                        (1ULL << IR_MOTOR_A_PWM) | (1ULL << IR_MOTOR_A_IN1) | (1ULL << IR_MOTOR_A_IN2) |
                        (1ULL << IR_MOTOR_B_PWM) | (1ULL << IR_MOTOR_B_IN1) | (1ULL << IR_MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_conf);

    ledc_timer_config_t timer_conf = {
        .speed_mode = IR_LEDC_MODE,
        .timer_num = IR_LEDC_TIMER,
        .duty_resolution = IR_PWM_RESOL,
        .freq_hz = IR_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t ch_conf = {
        .speed_mode = IR_LEDC_MODE,
        .timer_sel = IR_LEDC_TIMER,
        .duty = 0,
    };
    ch_conf.channel = IR_LEDC_CH_D; ch_conf.gpio_num = IR_MOTOR_D_PWM; ledc_channel_config(&ch_conf);
    ch_conf.channel = IR_LEDC_CH_A; ch_conf.gpio_num = IR_MOTOR_A_PWM; ledc_channel_config(&ch_conf);
    ch_conf.channel = IR_LEDC_CH_B; ch_conf.gpio_num = IR_MOTOR_B_PWM; ledc_channel_config(&ch_conf);
}

void ir_set_all_motors(const IrMotorSpeed *speed)
{
    set_motor(IR_MOTOR_D_PWM, IR_MOTOR_D_IN1, IR_MOTOR_D_IN2, speed->D, IR_LEDC_CH_D);
    set_motor(IR_MOTOR_A_PWM, IR_MOTOR_A_IN1, IR_MOTOR_A_IN2, speed->A, IR_LEDC_CH_A);
    set_motor(IR_MOTOR_B_PWM, IR_MOTOR_B_IN1, IR_MOTOR_B_IN2, speed->B, IR_LEDC_CH_B);
}

void ir_stop_motors(void)
{
    IrMotorSpeed zero = {0, 0, 0};
    ir_set_all_motors(&zero);
}
