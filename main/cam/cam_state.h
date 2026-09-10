#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "cam_types.h"

/* Shared globals (were file-scope static in cam_avoid_catch.c). */

extern LastDir g_last_dir;
extern Phase g_phase;
extern int g_lost_frames;
extern int g_decode_ms;
extern MotorSpeed g_last_wheels;
extern float g_last_vy;
extern float g_last_om;
extern float g_last_dist;
extern float g_current_distance;
extern int64_t g_phase_t0;
extern int g_hit_cm;
extern int g_t_white;
extern bool g_t_seen;
extern bool g_has_avoided;
extern volatile Mission g_mission;
extern int64_t g_wait_t0;

extern uint8_t *s_rgb;
extern uint8_t *s_mask;
extern uint8_t *s_visited;
extern uint8_t *s_jpeg_work;

extern SemaphoreHandle_t s_frame_mutex;
extern SemaphoreHandle_t s_frame_ready;
extern SemaphoreHandle_t s_dbg_mutex;
extern uint8_t *s_jpeg;
extern volatile uint32_t s_jpeg_len;
extern uint8_t *s_gray;
extern uint8_t *s_bin;
extern uint8_t *s_dbg_gray;
extern uint8_t *s_dbg_bin;
extern int s_img_w;
extern int s_img_h;
extern RowScan g_scan_rows[SCAN_ROWS];
extern int g_scan_y[SCAN_ROWS];
extern int g_poly_x[SCAN_ROWS];
extern int g_poly_y[SCAN_ROWS];
extern int g_poly_n;
extern Sight g_dbg_path;
extern int g_dbg_w;
extern int g_dbg_h;
extern char s_dbg_json[4096];
extern int s_dbg_json_len;
extern volatile bool s_dbg_ready;

extern AllWheelRPM g_current_rpm;

extern PushState g_state;
extern BallKind g_ball_kind;
extern int64_t g_stage_t0;
extern int g_lock_frames;
extern int64_t g_backup_until;
extern int64_t g_pulse_ready_at;
extern int g_stop_hold;
extern FrameSight g_sight;
extern BlobTarget g_locked_ball;
extern bool g_done_red;
extern bool g_done_blue;
extern float g_last_d_ball;
extern float g_last_d_net;
extern bool g_orbit_dir_valid;
extern bool g_orbit_left;
extern int g_last_abs_ldx;
extern int g_orbit_worse_frames;
extern FrameSight g_dbg_sight;
extern PushState g_dbg_state;
extern BallKind g_dbg_kind;
