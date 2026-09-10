#include "motor.h"
#include "cam_config.h"
#include "cam_state.h"
#include "util.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "AVOID_CATCH";

void motor_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << MOTOR_D_PWM) | (1ULL << MOTOR_D_IN1) | (1ULL << MOTOR_D_IN2) |
                        (1ULL << MOTOR_A_PWM) | (1ULL << MOTOR_A_IN1) | (1ULL << MOTOR_A_IN2) |
                        (1ULL << MOTOR_B_PWM) | (1ULL << MOTOR_B_IN1) | (1ULL << MOTOR_B_IN2),
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = PWM_RESOL,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
    };
    ch.channel = LEDC_CH_D; ch.gpio_num = MOTOR_D_PWM; ledc_channel_config(&ch);
    ch.channel = LEDC_CH_A; ch.gpio_num = MOTOR_A_PWM; ledc_channel_config(&ch);
    ch.channel = LEDC_CH_B; ch.gpio_num = MOTOR_B_PWM; ledc_channel_config(&ch);
}

static void set_motor(gpio_num_t in1, gpio_num_t in2, float speed, ledc_channel_t ch)
{
    int pwm = (int)roundf(speed);
    if (pwm > MAX_SPEED) pwm = MAX_SPEED;
    if (pwm < -MAX_SPEED) pwm = -MAX_SPEED;

    if (pwm > 0) {
        gpio_set_level(in1, 1);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, ch, pwm);
    } else if (pwm < 0) {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 1);
        ledc_set_duty(LEDC_MODE, ch, -pwm);
    } else {
        gpio_set_level(in1, 0);
        gpio_set_level(in2, 0);
        ledc_set_duty(LEDC_MODE, ch, 0);
    }
    ledc_update_duty(LEDC_MODE, ch);
}

void set_all_motors(const MotorSpeed *s)
{
    set_motor(MOTOR_D_IN1, MOTOR_D_IN2, s->D, LEDC_CH_D);
    set_motor(MOTOR_A_IN1, MOTOR_A_IN2, s->A, LEDC_CH_A);
    set_motor(MOTOR_B_IN1, MOTOR_B_IN2, s->B, LEDC_CH_B);
}

void stop_motors(void)
{
    MotorSpeed z = {0, 0, 0};
    set_all_motors(&z);
}
MotorSpeed inverse_kinematics(const Velocity *vel)
{
    MotorSpeed w;
    float L = WHEEL_DISTANCE;
    float raw_D = -SIN_60 * vel->vx + COS_60 * vel->vy + L * vel->omega;
    float raw_A =  SIN_60 * vel->vx + COS_60 * vel->vy - L * vel->omega;
    float raw_B = vel->vx + L * vel->omega;
    bool is_strafe = (fabsf(vel->vx) > 0.001f) && (fabsf(vel->vy) < 0.001f) && (fabsf(vel->omega) < 0.001f);
    if (is_strafe) {
        w.D = raw_D * STRAFE_SCALE_D;
        w.A = raw_A * STRAFE_SCALE_A;
        w.B = raw_B * STRAFE_SCALE_B;
    } else {
        w.D = raw_D;
        w.A = raw_A;
        w.B = raw_B;
    }
    w.D = clampf(w.D, -(float)PWM_CAP, (float)PWM_CAP);
    w.A = clampf(w.A, -(float)PWM_CAP, (float)PWM_CAP);
    w.B = clampf(w.B, -(float)PWM_CAP, (float)PWM_CAP);
    return w;
}

void apply_vel(const Velocity *vel)
{
    MotorSpeed s = inverse_kinematics(vel);
    g_last_vy = vel->vy;
    g_last_om = vel->omega;
    g_last_wheels = s;
    set_all_motors(&s);
}

void strafe(float vx)
{
    Velocity v = { .vx = vx, .vy = 0.0f, .omega = 0.0f };
    apply_vel(&v);
}

/* 直行差速：B 只辅助一点点。自转不用这个。 */
void drive(float vy, float omega)
{
    float fwd = 0.0f;
    if (vy > 1.0f) {
        fwd = clampf(vy, 0.0f, (float)PWM_CAP);
    }
    float rot = clampf(omega, -ROT_MAX, ROT_MAX);

    MotorSpeed s;
    s.D = clampf(fwd + rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.A = clampf(fwd - rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.B = clampf(rot * 0.70f, -(float)PWM_CAP, (float)PWM_CAP);

    g_last_vy = vy;
    g_last_om = rot;
    g_last_wheels = s;
    set_all_motors(&s);
}

/* 三轮同速原地转：右前 A 反向，D/B 同向 */
void spin_in_place(bool left)
{
    float s = left ? -SPIN_PWM : SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    g_last_wheels = m;
    set_all_motors(&m);
}
MotorSpeed catch_make_drive(float vy, float omega)
{
    float fwd = 0.0f;
    if (fabsf(vy) > 1.0f) {
        fwd = clampf(vy, -(float)PWM_CAP, (float)PWM_CAP);
    }
    float rot = clampf(omega, -CATCH_ROT_MAX, CATCH_ROT_MAX);

    MotorSpeed s;
    s.D = clampf(fwd + rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.A = clampf(fwd - rot, -(float)PWM_CAP, (float)PWM_CAP);
    s.B = clampf(rot * 0.70f, -(float)PWM_CAP, (float)PWM_CAP);
    g_last_vy = vy;
    g_last_om = rot;
    return s;
}

void catch_drive(float vy, float omega)
{
    MotorSpeed s = catch_make_drive(vy, omega);
    g_last_wheels = s;
    set_all_motors(&s);
}

int pulse_brake_ms(int on_ms)
{
    /* 约正向时长的 1/5，且不超过上限，保证刹停而不反转 */
    int b = on_ms / 5;
    if (b < 1) {
        b = 1;
    }
    if (b > PULSE_BRAKE_MAX_MS) {
        b = PULSE_BRAKE_MAX_MS;
    }
    if (b >= on_ms) {
        b = (on_ms > 1) ? (on_ms / 2) : 1;
    }
    return b;
}

MotorSpeed pulse_brake_cmd(const MotorSpeed *cmd)
{
    float s = PULSE_BRAKE_SCALE;
    MotorSpeed b;
    b.D = clampf(-cmd->D * s, -(float)PWM_CAP, (float)PWM_CAP);
    b.A = clampf(-cmd->A * s, -(float)PWM_CAP, (float)PWM_CAP);
    b.B = clampf(-cmd->B * s, -(float)PWM_CAP, (float)PWM_CAP);
    return b;
}

void wait_pulse_us(int64_t us)
{
    int64_t t0 = esp_timer_get_time();
    while (esp_timer_get_time() - t0 < us) {
        vTaskDelay(1);
    }
}

/* 真按时长通电→制动→停车。不再把电机一直开到下一帧。 */
void pulse_apply_ms(const MotorSpeed *cmd, int on_ms, int off_ms)
{
    int64_t now = esp_timer_get_time();
    if (now < g_pulse_ready_at) {
        stop_motors();
        return;
    }
    if (on_ms < 1) {
        on_ms = 1;
    }
    if (off_ms < 0) {
        off_ms = 0;
    }

    g_last_wheels = *cmd;
    set_all_motors(cmd);
    wait_pulse_us((int64_t)on_ms * 1000);

    int brake_ms = pulse_brake_ms(on_ms);
    MotorSpeed brk = pulse_brake_cmd(cmd);
    set_all_motors(&brk);
    wait_pulse_us((int64_t)brake_ms * 1000);
    stop_motors();
    g_last_wheels = *cmd;
    g_pulse_ready_at = esp_timer_get_time() + (int64_t)off_ms * 1000;
}

void pulse_apply(const MotorSpeed *cmd)
{
    pulse_apply_ms(cmd, PULSE_ON_MS, PULSE_OFF_MS);
}

void pulse_drive(float vy, float omega)
{
    MotorSpeed s = catch_make_drive(vy, omega);
    pulse_apply(&s);
}

void pulse_drive_ms(float vy, float omega, int on_ms)
{
    MotorSpeed s = catch_make_drive(vy, omega);
    pulse_apply_ms(&s, on_ms, PULSE_OFF_MS);
}

void catch_spin_in_place(bool left)
{
    float s = left ? -CATCH_SPIN_PWM : CATCH_SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    /* 转完立刻停，下一帧再判断，避免连转大半圈 */
    pulse_apply_ms(&m, PULSE_SPIN_ON_MS, 0);
}

/* 仅找球：50ms 通电 + 固定 5ms 反刹。其它原地转仍走 catch_spin_in_place。 */
void catch_spin_search_ball(bool left)
{
    float s = left ? -CATCH_SPIN_PWM : CATCH_SPIN_PWM;
    s = clampf(s, -(float)PWM_CAP, (float)PWM_CAP);
    MotorSpeed m = { s, -s, s };
    g_last_vy = 0.0f;
    g_last_om = s;
    g_last_wheels = m;

    int64_t now = esp_timer_get_time();
    if (now < g_pulse_ready_at) {
        stop_motors();
        return;
    }
    set_all_motors(&m);
    wait_pulse_us((int64_t)SEARCH_SPIN_ON_MS * 1000);
    MotorSpeed brk = pulse_brake_cmd(&m);
    set_all_motors(&brk);
    wait_pulse_us((int64_t)SEARCH_SPIN_BRAKE_MS * 1000);
    stop_motors();
    g_last_wheels = m;
    g_pulse_ready_at = 0;
}

/* 绕车头前方点公转，配速与 orbit_test.c 的 catch_orbit_cmd 相同 */
MotorSpeed catch_orbit_cmd(bool left)
{
    float vx = left ? CATCH_ORBIT_VX : -CATCH_ORBIT_VX;
    MotorSpeed m;
    m.D = clampf(-CATCH_ORBIT_SIN60 * vx * CATCH_ORBIT_SCALE_D, -(float)PWM_CAP, (float)PWM_CAP);
    m.A = clampf( CATCH_ORBIT_SIN60 * vx * CATCH_ORBIT_SCALE_A, -(float)PWM_CAP, (float)PWM_CAP);
    m.B = clampf(vx * CATCH_ORBIT_SCALE_B * CATCH_ORBIT_B_BOOST,
                 -(float)PWM_CAP, (float)PWM_CAP);
    return m;
}

/* 仅 ST_ALIGN：70ms 通电 + 固定 5ms 反刹，不走 pulse_apply_ms */
void pulse_orbit_around_front(bool left)
{
    MotorSpeed m = catch_orbit_cmd(left);
    g_last_vy = 0.0f;
    g_last_om = m.B;
    g_last_wheels = m;

    int64_t now = esp_timer_get_time();
    if (now < g_pulse_ready_at) {
        stop_motors();
        return;
    }

    int on_ms = CATCH_ORBIT_PULSE_MS;
    if (on_ms < 1) {
        on_ms = 1;
    }
    set_all_motors(&m);
    wait_pulse_us((int64_t)on_ms * 1000);

    MotorSpeed brk = pulse_brake_cmd(&m);
    set_all_motors(&brk);
    wait_pulse_us((int64_t)CATCH_ORBIT_BRAKE_MS * 1000);

    stop_motors();
    g_last_wheels = m;
    g_pulse_ready_at = 0;
}

/* 网在球右侧(ldx>0)时左绕：实车日志里左绕(om>0)会让网的 cx 变小。
 * |ldx| 变小保持原方向；连续两帧变大则翻转，不能再赋一次 want_left（符号错时永远拧反）。 */
bool align_orbit_left(int ldx)
{
    int al = abs(ldx);
    bool want_left = (ldx > 0);
    if (!g_orbit_dir_valid) {
        g_orbit_left = want_left;
        g_orbit_dir_valid = true;
        g_orbit_worse_frames = 0;
    } else if (g_last_abs_ldx >= 0) {
        if (al < g_last_abs_ldx) {
            g_orbit_worse_frames = 0;
        } else if (al > g_last_abs_ldx) {
            g_orbit_worse_frames++;
            if (g_orbit_worse_frames >= 2) {
                g_orbit_left = !g_orbit_left;
                g_orbit_worse_frames = 0;
                ESP_LOGI(TAG, "ALIGN reverse orbit left=%d |ldx|=%d", (int)g_orbit_left, al);
            }
        }
    }
    g_last_abs_ldx = al;
    return g_orbit_left;
}
