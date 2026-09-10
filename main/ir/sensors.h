/**
 * @file sensors.h
 * @brief 红外循迹传感器 + 超声波（红外工程）
 */
#pragma once

#include "ir_types.h"

void ir_sensor_init(void);
void ir_read_sensors(IrSensorData *data);
int  ir_get_sensor_code(const IrSensorData *data);

void  ir_ultrasonic_init(void);
float ir_get_ultrasonic_distance(void);

/** GUI 可读的最近一次有效距离 */
float ir_telemetry_distance(void);
