/**
 * @file ir_config.h
 * @brief 红外循迹工程：引脚 + 超参数（调参唯一入口）
 *
 * 与摄像头工程(cam/)完全独立，请勿混用。
 */
#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

/* ======================== 红外循迹传感器引脚 ======================== */
#define IR_SENSOR_L2_PIN        GPIO_NUM_4
#define IR_SENSOR_L1_PIN        GPIO_NUM_5
#define IR_SENSOR_R1_PIN        GPIO_NUM_6
#define IR_SENSOR_R2_PIN        GPIO_NUM_7

/* ======================== 电机 PWM & 方向 ======================== */
/* 左前 (D) */
#define IR_MOTOR_D_PWM          GPIO_NUM_14
#define IR_MOTOR_D_IN1          GPIO_NUM_13
#define IR_MOTOR_D_IN2          GPIO_NUM_12
/* 右前 (A) */
#define IR_MOTOR_A_PWM          GPIO_NUM_21
#define IR_MOTOR_A_IN1          GPIO_NUM_46
#define IR_MOTOR_A_IN2          GPIO_NUM_3
/* 后轮 (B) */
#define IR_MOTOR_B_PWM          GPIO_NUM_15
#define IR_MOTOR_B_IN1          GPIO_NUM_16
#define IR_MOTOR_B_IN2          GPIO_NUM_17

/* ======================== 霍尔编码器 ======================== */
#define IR_HALL_D_A_PIN         GPIO_NUM_10
#define IR_HALL_D_B_PIN         GPIO_NUM_11
#define IR_HALL_A_A_PIN         GPIO_NUM_20
#define IR_HALL_A_B_PIN         GPIO_NUM_19
#define IR_HALL_B_A_PIN         GPIO_NUM_42
#define IR_HALL_B_B_PIN         GPIO_NUM_41

#define IR_ENCODER_PPR          512
#define IR_SAMPLE_TIME_MS       100

#define IR_WHEEL_D              0
#define IR_WHEEL_A              1
#define IR_WHEEL_B              2

/* ======================== 超声波 ======================== */
#define IR_TRIG_PIN             GPIO_NUM_8
#define IR_ECHO_PIN             GPIO_NUM_18
#define IR_TIMEOUT_US           30000

/* ======================== LCD (ST7789 128x160) ======================== */
#define IR_PIN_NUM_CS           GPIO_NUM_2
#define IR_PIN_NUM_SCK          GPIO_NUM_1
#define IR_PIN_NUM_SDI          GPIO_NUM_38
#define IR_PIN_NUM_DC           GPIO_NUM_39
#define IR_PIN_NUM_RST          GPIO_NUM_40
#define IR_LCD_H_RES            128
#define IR_LCD_V_RES            160

/* ======================== PWM / LEDC ======================== */
#define IR_LEDC_MODE            LEDC_LOW_SPEED_MODE
#define IR_LEDC_TIMER           LEDC_TIMER_0
#define IR_LEDC_CH_D            LEDC_CHANNEL_0
#define IR_LEDC_CH_A            LEDC_CHANNEL_1
#define IR_LEDC_CH_B            LEDC_CHANNEL_2
#define IR_PWM_FREQ             1000
#define IR_PWM_RESOL            8
#define IR_MAX_SPEED            255

/* ======================== 运动速度 / 周期 ======================== */
#define IR_BASE_SPEED           85
#define IR_STRAFE_SPEED         47
#define IR_CONTROL_PERIOD       3
#define IR_SEARCH_PERIOD        8

/* ======================== 运动学几何 ======================== */
#define IR_WHEEL_DISTANCE       0.1f
#define IR_SIN_60               0.8660254f
#define IR_COS_60               0.5f

/* ======================== 循迹修正参数 ======================== */
#define IR_MICRO_PULSE_MS       2
#define IR_MACRO_PULSE_MS       2.5
#define IR_OMEGA_MICRO          15.0f
#define IR_OMEGA_MACRO          18.0f
#define IR_OMEGA_SEARCH         310.0f

/* ======================== 轮速 Scale ======================== */
#define IR_STRAFE_SCALE_D       0.87f
#define IR_STRAFE_SCALE_A       0.87f
#define IR_STRAFE_SCALE_B       1.20f

#define IR_FWD_SCALE_D          1.00f
#define IR_FWD_SCALE_A          1.00f
#define IR_FWD_SCALE_B          1.00f

#define IR_ROT_SCALE_D          1.00f
#define IR_ROT_SCALE_A          1.00f
#define IR_ROT_SCALE_B          1.00f

/* ======================== 避障时序（原 magic number 抽成可调参） ======================== */
#define IR_AVOID_TRIGGER_CM     5.0f
#define IR_AVOID_CLEAR_CM       5.0f
#define IR_STRAFE_FORCE_MS      800
#define IR_STRAFE_POLL_MS       100
#define IR_STRAFE_EXTRA_MS      11
#define IR_STRAFE_STOP_DELAY_MS 400
#define IR_AVOID_FWD_MS         1530
#define IR_PHASE_STOP_DELAY_MS  200
#define IR_STRAFE_BACK_FACTOR   0.90f
#define IR_STRAFE_BACK_PULSE_MS 25
