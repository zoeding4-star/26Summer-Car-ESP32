#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "cam_types.h"

void motor_init(void);
void set_all_motors(const MotorSpeed *s);
void stop_motors(void);
MotorSpeed inverse_kinematics(const Velocity *vel);
void apply_vel(const Velocity *vel);
void strafe(float vx);
void drive(float vy, float omega);
void spin_in_place(bool left);

MotorSpeed catch_make_drive(float vy, float omega);
void catch_drive(float vy, float omega);
int pulse_brake_ms(int on_ms);
MotorSpeed pulse_brake_cmd(const MotorSpeed *cmd);
void wait_pulse_us(int64_t us);
void pulse_apply_ms(const MotorSpeed *cmd, int on_ms, int off_ms);
void pulse_apply(const MotorSpeed *cmd);
void pulse_drive(float vy, float omega);
void pulse_drive_ms(float vy, float omega, int on_ms);
void catch_spin_in_place(bool left);
void catch_spin_search_ball(bool left);
MotorSpeed catch_orbit_cmd(bool left);
void pulse_orbit_around_front(bool left);
bool align_orbit_left(int ldx);
