/**
 * @file kinematics.h
 * @brief 全向轮逆解与脉冲执行（红外工程）
 */
#pragma once

#include "ir_types.h"

IrMotorSpeed ir_inverse_kinematics(const IrVelocity *vel);
void ir_execute_pulse(IrVelocity vel, int duration_ms);
