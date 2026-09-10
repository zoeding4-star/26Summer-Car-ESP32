#include "vision_line.h"
#include "cam_config.h"
#include "cam_state.h"
#include "decode.h"
#include "debug_wifi.h"
#include "motor.h"
#include "sensing.h"
#include "util.h"

#include "esp_log.h"
#include "esp_timer.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "AVOID_CATCH";

int roi_x0(void)
{
    int x = scaled_px(ROI_X0_480);
    return (x < 0) ? 0 : x;
}

int roi_x1(void)
{
    int x = scaled_px(ROI_X1_480);
    if (x >= s_img_w) {
        x = s_img_w - 1;
    }
    return x;
}

int max_line_w(void)
{
    int w = scaled_px(MAX_LINE_W * CAM_WIDTH / 120);
    if (w < 4) {
        w = 4;
    }
    return w;
}

static void scan_row(int y, RowScan *out)
{
    memset(out, 0, sizeof(*out));
    int x0 = roi_x0();
    int x1 = roi_x1();
    int span = x1 - x0 + 1;
    if (span < 4) {
        return;
    }
    out->span = span;

    int sum = 0;
    for (int x = x0; x <= x1; x++) {
        sum += luma_at(x, y);
    }
    int mean = sum / span;
    int th = mean - 18;
    if (th < LINE_THRESH_MIN) {
        th = LINE_THRESH_MIN;
    }
    if (th > LINE_THRESH_MAX) {
        th = LINE_THRESH_MAX;
    }

    int run0 = -1;
    int black_n = 0;
    for (int x = x0; x <= x1 + 1; x++) {
        bool on = false;
        if (x <= x1) {
            on = luma_at(x, y) < th;
            s_bin[y * s_img_w + x] = on ? 1 : 0;
            if (on) {
                black_n++;
            }
        }
        if (on) {
            if (run0 < 0) {
                run0 = x;
            }
        } else if (run0 >= 0) {
            int w = x - run0;
            if (w >= MIN_LINE_W && out->n < MAX_BLOBS) {
                Blob *b = &out->b[out->n++];
                b->left = run0;
                b->right = x - 1;
                b->width = w;
                b->cx = (run0 + x - 1) / 2;
                b->mass = w;
            }
            run0 = -1;
        }
    }
    out->black_n = black_n;
    out->full_bar = (black_n * 100 >= span * BAR_FRAC);
}

int pick_stem_cx(const RowScan *row, int pred, int max_jump)
{
    int best = -1;
    int best_d = 10000;
    int cap = max_line_w();
    for (int i = 0; i < row->n; i++) {
        if (row->b[i].width > cap && !row->full_bar) {
            continue;
        }
        if (row->full_bar) {
            continue;
        }
        int d = abs(row->b[i].cx - pred);
        if (d < best_d && d <= max_jump) {
            best_d = d;
            best = row->b[i].cx;
        }
    }
    return best;
}

void probe_sides(int cx, int y, int *left, int *right)
{
    int x0 = roi_x0();
    int x1 = roi_x1();
    int L = 0, R = 0;
    for (int x = x0; x <= x1; x++) {
        if (!s_bin[y * s_img_w + x]) {
            continue;
        }
        if (x < cx - 1) {
            L++;
        } else if (x > cx + 1) {
            R++;
        }
    }
    *left = L;
    *right = R;
}

const char *view_name(ViewType t)
{
    switch (t) {
    case VIEW_STEM: return "STEM";
    case VIEW_BAR:  return "BAR";
    default:        return "NONE";
    }
}

Sight look(void)
{
    Sight s;
    memset(&s, 0, sizeof(s));
    s.near_cx = -1;
    s.far_cx = -1;
    s.near_y = -1;
    s.far_y = -1;
    s.corner_x = -1;
    s.corner_y = -1;
    g_poly_n = 0;

    int h = s_img_h;
    int y_near = SCAN_Y_NEAR * h / CAM_HEIGHT;
    int y_far = SCAN_Y_FAR * h / CAM_HEIGHT;
    if (y_near >= h) {
        y_near = h - 1;
    }
    if (y_far < 0) {
        y_far = 0;
    }
    if (y_far >= y_near) {
        y_far = y_near - (SCAN_ROWS - 1);
        if (y_far < 0) {
            y_far = 0;
        }
    }

    int x0 = roi_x0();
    int x1 = roi_x1();
    for (int y = y_far; y <= y_near; y++) {
        memset(s_bin + y * s_img_w + x0, 0, (size_t)(x1 - x0 + 1));
    }

    for (int i = 0; i < SCAN_ROWS; i++) {
        int y = y_near - (y_near - y_far) * i / (SCAN_ROWS - 1);
        g_scan_y[i] = y;
        scan_row(y, &g_scan_rows[i]);
        if (g_scan_rows[i].black_n > 0) {
            s.has_black = true;
        }
    }

    int center = s_img_w / 2;
    int pred = center;
    int jump = scaled_px(STEM_MAX_JUMP * CAM_WIDTH / 120);
    if (jump < 4) {
        jump = 4;
    }
    int miss = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        int cx = pick_stem_cx(&g_scan_rows[i], pred, jump);
        if (cx < 0) {
            miss++;
            if (miss >= 2) {
                break;
            }
            continue;
        }
        miss = 0;
        g_poly_x[g_poly_n] = cx;
        g_poly_y[g_poly_n] = g_scan_y[i];
        g_poly_n++;
        pred = cx;
    }
    s.stem_n = g_poly_n;

    if (g_poly_n >= 1) {
        s.near_ok = true;
        s.near_cx = g_poly_x[0];
        s.near_y = g_poly_y[0];
        s.far_cx = g_poly_x[g_poly_n - 1];
        s.far_y = g_poly_y[g_poly_n - 1];
        s.offset = s.near_cx - center;
        if (g_poly_n >= 2) {
            s.angle = g_poly_x[0] - g_poly_x[1];
        } else {
            s.angle = s.offset;
        }
        s.type = VIEW_STEM;
    }

    int stem_cx = (g_poly_n >= 1) ? g_poly_x[0] : center;
    int Lall = 0, Rall = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        int L = 0, R = 0;
        probe_sides(stem_cx, g_scan_y[i], &L, &R);
        Lall += L;
        Rall += R;
        if (g_scan_rows[i].full_bar) {
            if (L > R * 1.2f) {
                s.turn_left = true;
            } else if (R > L * 1.2f) {
                s.turn_right = true;
            }
        }
        for (int k = 0; k < g_scan_rows[i].n; k++) {
            if (g_scan_rows[i].b[k].width >= max_line_w()) {
                if (g_scan_rows[i].b[k].cx < stem_cx) {
                    s.turn_left = true;
                } else if (g_scan_rows[i].b[k].cx > stem_cx) {
                    s.turn_right = true;
                }
            }
        }
    }
    s.left_mass = Lall;
    s.right_mass = Rall;

    bool wide_l = false;
    bool wide_r = false;
    int cap = max_line_w();
    int near_bars = 0;
    for (int i = 0; i < SCAN_ROWS; i++) {
        if (g_scan_rows[i].full_bar) {
            if (i < 3) {
                near_bars++;
            }
            int L = 0, R = 0;
            probe_sides(stem_cx, g_scan_y[i], &L, &R);
            if (L > 0) {
                wide_l = true;
            }
            if (R > 0) {
                wide_r = true;
            }
            if (L == 0 && R == 0) {
                wide_l = true;
                wide_r = true;
            }
        }
        for (int k = 0; k < g_scan_rows[i].n; k++) {
            if (g_scan_rows[i].b[k].width >= cap) {
                if (g_scan_rows[i].b[k].cx <= stem_cx) {
                    wide_l = true;
                }
                if (g_scan_rows[i].b[k].cx >= stem_cx) {
                    wide_r = true;
                }
            }
        }
    }
    /* 左或右有一大片，或两边都有：都算 T */
    if (wide_l || wide_r || near_bars >= 1) {
        s.t_bar = true;
    }

    if (Lall > (int)(Rall * SIDE_RATIO) && Lall >= MIN_SIDE_MASS) {
        s.turn_left = true;
    }
    if (Rall > (int)(Lall * SIDE_RATIO) && Rall >= MIN_SIDE_MASS) {
        s.turn_right = true;
    }

    /* 折线：最低端往上方向突变 */
    int kink = scaled_px(KINK_PX * CAM_WIDTH / 60);
    if (kink < 2) {
        kink = 2;
    }
    if (g_poly_n >= 3) {
        for (int i = 1; i < g_poly_n - 1; i++) {
            int a = g_poly_x[i] - g_poly_x[i - 1];
            int b = g_poly_x[i + 1] - g_poly_x[i];
            if (abs(a - b) >= kink && (a * b < 0 || abs(b) >= kink)) {
                s.kink = abs(a - b);
                s.corner_x = g_poly_x[i];
                s.corner_y = g_poly_y[i];
                if (b < 0 || (b == 0 && a > 0)) {
                    s.turn_left = true;
                } else {
                    s.turn_right = true;
                }
            }
        }
    }

    if (s.turn_left && s.turn_right) {
        s.t_bar = true;
        s.turn_left = false;
        s.turn_right = false;
    }

    if (!s.near_ok && s.has_black) {
        s.type = VIEW_BAR;
        if (s.t_bar) {
            /* T：不再当成单侧直角 */
        } else if (!s.turn_left && !s.turn_right) {
            if (Lall >= Rall) {
                s.turn_left = true;
            } else {
                s.turn_right = true;
            }
        }
    }
    if (!s.has_black) {
        s.type = VIEW_NONE;
    }
    return s;
}
void follow_stem(const Sight *p)
{
    int dead = scaled_px(DEAD_PX_480);
    int macro = scaled_px(MACRO_PX_480);
    int err = p->angle;
    if (abs(err) <= dead) {
        err = p->offset;
    }
    if (abs(err) <= dead) {
        drive(BASE_SPEED, 0.0f);
        return;
    }
    float om = (abs(err) <= macro) ? OMEGA_MICRO : OMEGA_MACRO;
    if (err < 0) {
        om = -om;
    }
    drive(BASE_SPEED, om);
}

bool heading_ok(const Sight *p)
{
    int dead = scaled_px(DEAD_PX_480);
    int off = scaled_px(14);
    return p->near_ok && abs(p->angle) <= dead && abs(p->offset) <= off;
}

void begin_phase(Phase ph)
{
    g_phase = ph;
    g_phase_t0 = esp_timer_get_time();
}

void align_heading(const Sight *p)
{
    int err = p->near_ok ? ((abs(p->angle) > 0) ? p->angle : p->offset) : 0;
    if (!p->near_ok) {
        spin_in_place(g_last_dir == LAST_DIR_LEFT);
        return;
    }
    if (abs(err) <= scaled_px(DEAD_PX_480)) {
        drive(0.0f, 0.0f);
        return;
    }
    /* 不前进时差速太小转不动，改用原地转摆正 */
    spin_in_place(err < 0);
}

void follow_tick(const Sight *p)
{
    if (g_has_avoided && (p->t_bar || p->type == VIEW_BAR)) {
        g_t_seen = true;
        g_t_white = 0;
        drive(BASE_SPEED, 0.0f);
        return;
    }
    /* 避障前：只有左右都是横带的十字才当 T 直行；单侧仍记直角 */
    if (!g_has_avoided && p->t_bar && !p->turn_left && !p->turn_right) {
        drive(BASE_SPEED, 0.0f);
        return;
    }
    if (p->turn_left) {
        g_last_dir = LAST_DIR_LEFT;
    } else if (p->turn_right) {
        g_last_dir = LAST_DIR_RIGHT;
    }

    if (g_t_seen) {
        if (p->type == VIEW_NONE) {
            g_t_white++;
            if (g_t_white >= T_WHITE_FRAMES) {
                stop_motors();
                begin_phase(PHASE_STOP);
                ESP_LOGI(TAG, "终止线：全黑后全白，停车");
                return;
            }
            drive(BASE_SPEED, 0.0f);
            return;
        }
        g_t_white = 0;
        drive(BASE_SPEED, 0.0f);
        return;
    }

    if (p->type == VIEW_NONE) {
        g_lost_frames++;
        if (g_lost_frames > LOST_STOP_FRAMES) {
            stop_motors();
            ESP_LOGE(TAG, "连续丢线，停车保护");
            return;
        }
        spin_in_place(g_last_dir == LAST_DIR_LEFT);
        return;
    }

    g_lost_frames = 0;
    if (p->type == VIEW_STEM) {
        follow_stem(p);
    } else {
        drive(BASE_SPEED, 0.0f);
    }
}

void control_once(void)
{
    Sight p = look();
    float dist = ultrasonic_cm();

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG, "ph=%d %s stem=%d ang=%d off=%d dist=%.1f t=%d vy=%.0f om=%.0f %dms",
                 (int)g_phase, view_name(p.type), p.stem_n, p.angle, p.offset,
                 dist, (int)g_t_seen, g_last_vy, g_last_om, g_decode_ms);
        last_log = now;
    }

    switch (g_phase) {
    case PHASE_FOLLOW:
        if (!g_t_seen && dist > 0.0f && dist <= AVOID_TRIGGER_CM) {
            g_hit_cm++;
        } else {
            g_hit_cm = 0;
        }
        if (g_hit_cm >= 2) {
            ESP_LOGW(TAG, "障碍 %.1f cm，先摆正再横移", dist);
            g_hit_cm = 0;
            begin_phase(PHASE_ALIGN);
            align_heading(&p);
            break;
        }
        follow_tick(&p);
        break;

    case PHASE_ALIGN:
        if (heading_ok(&p) || (now - g_phase_t0) > (int64_t)STRAFE_ALIGN_MS * 1000) {
            ESP_LOGI(TAG, "开始横移 (朝向%s)", heading_ok(&p) ? "已正" : "超时");
            begin_phase(PHASE_STRAFE_FORCE);
            strafe(STRAFE_SPEED);
        } else {
            align_heading(&p);
        }
        break;

    case PHASE_STRAFE_FORCE:
        strafe(STRAFE_SPEED);
        if ((now - g_phase_t0) > (int64_t)STRAFE_FORCE_MS * 1000) {
            begin_phase(PHASE_STRAFE_WAIT);
        }
        break;

    case PHASE_STRAFE_WAIT:
        strafe(STRAFE_SPEED);
        if (dist < 0.0f || dist >= AVOID_CLEAR_CM) {
            ESP_LOGI(TAG, "前方已空 %.1f cm，直行绕过", dist);
            begin_phase(PHASE_FWD);
            drive(BASE_SPEED, 0.0f);
        }
        break;

    case PHASE_FWD:
        drive(BASE_SPEED, 0.0f);
        if ((now - g_phase_t0) > (int64_t)AVOID_FWD_MS * 1000) {
            ESP_LOGI(TAG, "反向横移找线");
            begin_phase(PHASE_STRAFE_BACK);
            strafe(-STRAFE_BACK_SPEED);
        }
        break;

    case PHASE_STRAFE_BACK:
        strafe(-STRAFE_BACK_SPEED);
        if (p.has_black || p.near_ok) {
            stop_motors();
            ESP_LOGI(TAG, "重新看到黑线，恢复循迹");
            g_has_avoided = true;
            g_lost_frames = 0;
            begin_phase(PHASE_FOLLOW);
        } else if ((now - g_phase_t0) > 4000000) {
            ESP_LOGW(TAG, "回线超时，停车");
            stop_motors();
            begin_phase(PHASE_STOP);
        }
        break;

    case PHASE_STOP:
        stop_motors();
        if (g_mission == MISSION_LINE) {
            g_mission = MISSION_WAIT;
            g_wait_t0 = esp_timer_get_time();
            ESP_LOGI(TAG, "循迹结束，等待 3 秒后推球");
        }
        break;
    }

    debug_update(&p);
}
