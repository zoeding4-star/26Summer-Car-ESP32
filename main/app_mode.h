/**
 * @file app_mode.h
 * @brief 说明：实际切换请改 main/CMakeLists.txt 里的 ROBOT_APP_MODE
 *
 *   "ir"  — 红外循迹 + 超声波避障（main/ir/，参数见 ir/ir_config.h）
 *   "cam" — 摄像头循迹 + 避障 + 找球（main/cam/，参数见 cam/cam_config.h）
 *
 * 两套工程完全并列，引脚与超参数互不统一。
 */
#pragma once
