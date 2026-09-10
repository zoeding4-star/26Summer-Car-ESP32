#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"

/* Tunable macros from cam_avoid_catch.c — values unchanged. */

/* ==================== 电机 ==================== */
#define MOTOR_D_PWM     GPIO_NUM_14
#define MOTOR_D_IN1     GPIO_NUM_13
#define MOTOR_D_IN2     GPIO_NUM_12
#define MOTOR_A_PWM     GPIO_NUM_21
#define MOTOR_A_IN1     GPIO_NUM_46
#define MOTOR_A_IN2     GPIO_NUM_3
#define MOTOR_B_PWM     GPIO_NUM_15
#define MOTOR_B_IN1     GPIO_NUM_16
#define MOTOR_B_IN2     GPIO_NUM_17

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_CH_D       LEDC_CHANNEL_0
#define LEDC_CH_A       LEDC_CHANNEL_1
#define LEDC_CH_B       LEDC_CHANNEL_2
#define PWM_FREQ        16000
#define PWM_RESOL       8
#define MAX_SPEED       255
#define PWM_CAP         66

#define TRIG_PIN        GPIO_NUM_8
#define ECHO_PIN        GPIO_NUM_18
#define TIMEOUT_US      30000

/* 霍尔：D/B 与 main_final 相同；A 避开 USB 的 19/20 */
#define HALL_D_A_PIN    GPIO_NUM_10
#define HALL_D_B_PIN    GPIO_NUM_11
#define HALL_A_A_PIN    GPIO_NUM_4
#define HALL_A_B_PIN    GPIO_NUM_5
#define HALL_B_A_PIN    GPIO_NUM_42
#define HALL_B_B_PIN    GPIO_NUM_41
#define ENCODER_PPR     512
#define SAMPLE_TIME_MS  100
#define WHEEL_D         0
#define WHEEL_A         1
#define WHEEL_B         2

/* ST7789 128x160，与 main_final 相同 */
#define PIN_NUM_CS      GPIO_NUM_2
#define PIN_NUM_SCK     GPIO_NUM_1
#define PIN_NUM_SDI     GPIO_NUM_38
#define PIN_NUM_DC      GPIO_NUM_39
#define PIN_NUM_RST     GPIO_NUM_40
#define LCD_H_RES       128
#define LCD_V_RES       160

#define WHEEL_DISTANCE  0.1f
#define SIN_60          0.8660254f
#define COS_60          0.5f
#define STRAFE_SCALE_D  0.87f
#define STRAFE_SCALE_A  0.87f
#define STRAFE_SCALE_B  1.88f     /* 后轮再快一点，抵消左后斜 */

/* ==================== 摄像头 ==================== */
#define CAM_WIDTH           480
#define CAM_HEIGHT          320
#define CAM_FPS             30
#define JPEG_DSCALE         3       /* 1/8，把解码从 ~100ms 打到几十 ms */
#define JPEG_XFER_SIZE      (88 * 1024)
#define CAM_FLIP_UD         1
#define CAM_FLIP_LR         1

/* 检测窗口为上一版的 5/4：底边仍贴车头，5 行=4 层 */
#define SCAN_Y_NEAR         320
#define SCAN_Y_FAR          253
#define SCAN_ROWS           5
#define ROI_X0_480          190
#define ROI_X1_480          290

#define LINE_THRESH_MIN     28
#define LINE_THRESH_MAX     140
#define MIN_LINE_W          1
#define MAX_LINE_W          18      /* 1/8 图上竖线很窄；更宽当横带 */
#define MAX_BLOBS           4
#define STEM_MAX_JUMP       8
#define BAR_FRAC            40      /* 一行黑像素占 ROI 百分比，视为横带 */
#define SIDE_RATIO          2.2f
#define MIN_SIDE_MASS       6
#define KINK_PX             3

/* 循迹慢而匀速；36 左右才能克服静摩擦 */
#define BASE_SPEED          36.0f
#define OMEGA_MICRO         5.0f
#define OMEGA_MACRO         10.0f
#define ROT_MAX             24.0f
#define SPIN_PWM            31.0f
#define DEAD_PX_480         6
#define MACRO_PX_480        16
#define LOST_STOP_FRAMES    180

#define STRAFE_SPEED        48.0f
#define STRAFE_BACK_SPEED   38.0f   /* 与避障横移同一套轮速比例 */
#define AVOID_TRIGGER_CM    10.0f
#define AVOID_CLEAR_CM      13.0f
#define STRAFE_FORCE_MS     800
#define STRAFE_ALIGN_MS     700
#define AVOID_FWD_MS        1485    /* 1350 再多 1/10 */
#define T_WHITE_FRAMES      3
#define CATCH_WAIT_MS       3000

#define JPEG_RGB_WORK_SZ    (16 * 1024)

/* ========== catch 推球 ========== */
#define APPROACH_SPEED  40.0f   /* 点动脉冲幅值，不是连续走 */
#define PUSH_SPEED      64.0f   /* 快速撞球 */
#define BACKUP_SPEED    36.0f   /* 匀速倒退 */
#define CATCH_OMEGA_MICRO  12.0f
#define CATCH_OMEGA_MACRO  18.0f
#define CATCH_SPIN_PWM   42.0f
#define CATCH_ROT_MAX     22.0f
/* 仅 ST_ALIGN 两个 pivot 使用，与 orbit_test.c 一致，勿改搜球/对中宏 */
#define CATCH_ORBIT_SIN60       0.8660254f
#define CATCH_ORBIT_SCALE_D     0.4f
#define CATCH_ORBIT_SCALE_A     0.4f
#define CATCH_ORBIT_SCALE_B     3.0f
#define CATCH_ORBIT_B_BOOST     1.00f
#define CATCH_ORBIT_VX          66.0f
#define CATCH_ORBIT_PULSE_MS    70
#define CATCH_ORBIT_BRAKE_MS    5       /* 仅绕球点动；勿改搜球制动 */
#define PULSE_ON_MS     120     /* 点动时长 ×4 */
#define PULSE_SPIN_ON_MS 20     /* 对中/丢球等：20ms 后立刻停 */
#define SEARCH_SPIN_ON_MS 50    /* 仅 ST_SEARCH_BALL 找球 */
#define SEARCH_SPIN_BRAKE_MS 5
#define PULSE_BACKUP_ON_MS 220  /* 后退每下比靠近走得更远 */
#define PULSE_OFF_MS    80      /* 前进/后退点动间隔；搜球不等这段 */
#define PULSE_BRAKE_SCALE 0.32f /* 反向制动幅值，低于起步静摩擦 */
#define PULSE_BRAKE_MAX_MS 24   /* 制动必须远短于正向，避免反走 */
#define PUSH_MS         500     /* 快速撞击时长 */
#define BACKUP_MS       3000    /* 匀速倒退时长 */
#define APPROACH_STOP_CM 8.0f
#define STAGE_TIMEOUT_US    (18LL * 1000 * 1000)
#define LOCK_HOLD_FRAMES    4
#define STOP_HOLD_FRAMES    3
#define ALIGN_HOLD_FRAMES   5
#define COINCIDE_PX         8
#define CENTER_DEAD_PX      6
#define CENTER_MACRO_PX     18
#define ALIGN_VERT_PX       4       /* 解码图上球-网 cx 差须同时满足 */
#define ALIGN_OK_DEG        10.0f   /* 与 ALIGN_VERT_PX 同时满足才快射 */
#define RETURN_LINE_MS      1500

#define MAX_CC_BLOBS        12
#define FLOOD_STACK         4096
#define MIN_BALL_AREA       12
#define MAX_BALL_AREA       2800
#define MIN_NET_AREA        8
#define MAX_NET_AREA        6000
#define MIN_CIRCULARITY     0.45f
#define MIN_CIRCULARITY_BLUE 0.32f
#define BALL_LOCK_RADIUS    14      /* 解码图半径阈值（1/4 尺度） */
#define BLACK_LINE_MIN_PCT  18
#define BALL_FAR_CROP_NUM   1       /* 找球时丢掉画面上方 1/8（远方） */
#define BALL_FAR_CROP_DEN   8
#define NEAR_Y_PCT          70      /* cy 超过画面 70% 视为贴到车头 */
#define NEAR_HOLD_FRAMES    4
