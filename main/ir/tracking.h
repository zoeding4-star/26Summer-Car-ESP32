/**
 * @file tracking.h
 * @brief 红外循迹控制环
 */
#pragma once

#include <stdbool.h>

/** 一次循迹迭代；终点到达返回 true */
bool ir_control_loop(int base_speed);

/** 避障完成后置位，使全黑传感器判定为终点 */
void ir_tracking_set_avoided(bool avoided);
bool ir_tracking_has_avoided(void);
