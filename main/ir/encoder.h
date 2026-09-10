/**
 * @file encoder.h
 * @brief 霍尔编码器测速（红外工程）
 */
#pragma once

#include "ir_types.h"

void ir_encoder_init(void);
IrAllWheelRPM ir_get_all_wheel_rpm(void);

/** GUI 可读的最近一次 RPM */
IrAllWheelRPM ir_telemetry_rpm(void);
