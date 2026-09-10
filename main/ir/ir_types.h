/**
 * @file ir_types.h
 * @brief 红外循迹工程公共类型
 */
#pragma once

#include <stdint.h>
#include "driver/pulse_cnt.h"

typedef struct {
    int l2, l1, r1, r2;
} IrSensorData;

typedef struct {
    float D, A, B;
} IrMotorSpeed;

typedef struct {
    float vx, vy, omega;
} IrVelocity;

typedef enum {
    IR_LAST_DIR_LEFT,
    IR_LAST_DIR_RIGHT
} IrLastDirection;

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel;
    int32_t pulse_count;
    int32_t last_count;
} IrEncoderHandle;

typedef struct {
    float rpm_D;
    float rpm_A;
    float rpm_B;
} IrAllWheelRPM;
