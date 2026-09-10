#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/pulse_cnt.h"
#include "cam_config.h"

typedef enum {
    MISSION_LINE = 0,
    MISSION_WAIT,
    MISSION_CATCH,
    MISSION_DONE
} Mission;

/* ==================== 类型 ==================== */
typedef struct {
    float D, A, B;
} MotorSpeed;

typedef struct {
    float vx, vy, omega;
} Velocity;

typedef enum {
    LAST_DIR_LEFT = 0,
    LAST_DIR_RIGHT
} LastDir;

typedef enum {
    PHASE_FOLLOW = 0,
    PHASE_ALIGN,
    PHASE_STRAFE_FORCE,
    PHASE_STRAFE_WAIT,
    PHASE_FWD,
    PHASE_STRAFE_BACK,
    PHASE_STOP
} Phase;

typedef enum {
    VIEW_NONE = 0,  /* 看不见黑线 */
    VIEW_STEM,      /* 有竖线，直行微调 */
    VIEW_BAR        /* 只有横带，还看见黑，先记方向再往前 */
} ViewType;

typedef struct {
    int left, right, cx, width, mass;
} Blob;

typedef struct {
    int n;
    Blob b[MAX_BLOBS];
    int black_n;
    int span;
    bool full_bar;
} RowScan;

typedef struct {
    ViewType type;
    int near_cx;
    int far_cx;
    int angle;       /* 最低端偏角：>0 底端偏右 */
    int offset;
    bool near_ok;
    bool has_black;
    bool turn_left;
    bool turn_right;
    bool t_bar;      /* 近端横着全黑且左右都有：终止 T */
    int left_mass;
    int right_mass;
    int stem_n;
    int kink;
    int near_y;
    int far_y;
    int corner_x;
    int corner_y;
} Sight;

typedef struct {
    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t channel;
} EncoderHandle_t;

typedef struct {
    float rpm_D;
    float rpm_A;
    float rpm_B;
} AllWheelRPM;

typedef enum {
    BALL_RED = 0,
    BALL_BLUE
} BallKind;

typedef enum {
    ST_IDLE = 0,
    ST_SEARCH_BALL,
    ST_APPROACH_BALL,
    ST_LOCK_BALL,
    ST_SEARCH_NET,
    ST_ALIGN,
    ST_PUSH,
    ST_BACKUP,
    ST_SEARCH_BLACK,
    ST_RETURN_END,
    ST_DONE,
    ST_FAIL
} PushState;

typedef enum {
    COLOR_RED = 0,
    COLOR_GREEN,
    COLOR_BLUE,
    COLOR_BLACK
} ColorId;

typedef struct {
    bool found;
    int cx, cy;
    int x0, y0, x1, y1;
    int area;
    int radius;
    float circularity;
    uint8_t mr, mg, mb;
    int mean_v;
} BlobTarget;

typedef struct {
    BlobTarget red;
    BlobTarget blue;
    BlobTarget ball;            /* 当前要推的球 */
    BlobTarget net;
    bool has_black_line;
    int black_cx;
    float align_deg;
} FrameSight;

typedef struct {
    int16_t x, y;
} Pt16;
