#pragma once

#include <stdbool.h>
#include "cam_types.h"

const char *state_name(PushState s);
const char *ball_name(BallKind k);
void enter_state(PushState st);
bool ready_to_shoot(const FrameSight *p);
void pulse_center_on_x(int tx, float speed);
bool stage_timeout(void);
void catch_debug_copy_images(void);
void catch_debug_update(const FrameSight *p);
void mark_ball_done(void);
void on_push_success(void);
void after_backup(void);
void catch_control_once(void);
