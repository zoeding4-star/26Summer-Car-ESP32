/**
 * @file motor.h
 * @brief 三轮电机驱动（红外工程）
 */
#pragma once

#include "ir_types.h"

void ir_motor_init(void);
void ir_set_all_motors(const IrMotorSpeed *speed);
void ir_stop_motors(void);
