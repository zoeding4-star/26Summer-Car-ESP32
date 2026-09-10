#include "catch_fsm.h"
#include "cam_config.h"
#include "cam_state.h"
#include "decode.h"
#include "motor.h"
#include "util.h"
#include "vision_catch.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char *TAG = "AVOID_CATCH";

const char *state_name(PushState s)
{
    switch (s) {
    case ST_IDLE:           return "IDLE";
    case ST_SEARCH_BALL:    return "SEARCH_BALL";
    case ST_APPROACH_BALL:  return "APPROACH_BALL";
    case ST_LOCK_BALL:      return "LOCK_BALL";
    case ST_SEARCH_NET:     return "SEARCH_NET";
    case ST_ALIGN:          return "ALIGN";
    case ST_PUSH:           return "PUSH";
    case ST_BACKUP:         return "BACKUP";
    case ST_SEARCH_BLACK:   return "SEARCH_BLACK";
    case ST_RETURN_END:     return "RETURN_END";
    case ST_DONE:           return "DONE";
    case ST_FAIL:           return "FAIL";
    default:                return "?";
    }
}

const char *ball_name(BallKind k)
{
    return (k == BALL_RED) ? "RED" : "BLUE";
}

void enter_state(PushState st)
{
    if (g_state != st) {
        ESP_LOGI(TAG, "STATE %s -> %s  ball=%s",
                 state_name(g_state), state_name(st), ball_name(g_ball_kind));
        g_pulse_ready_at = 0;
        g_stop_hold = 0;
        g_lock_frames = 0;
        if (st == ST_ALIGN) {
            g_orbit_dir_valid = false;
            g_last_abs_ldx = -1;
            g_orbit_worse_frames = 0;
        }
    }
    g_state = st;
    g_stage_t0 = esp_timer_get_time();
}
bool ready_to_shoot(const FrameSight *p)
{
    return line_almost_vertical(p) && ball_on_center(p);
}

/* ==================== 运动辅助 ==================== */
void pulse_center_on_x(int tx, float speed)
{
    int dead = center_dead_px();
    int macro = center_macro_px();
    int err = tx - s_img_w / 2;
    /* 贴球不再前进时：用搜球同款 2ms 旋转点动对中，避免 30ms 差速猛拧 */
    if (fabsf(speed) <= 1.0f) {
        if (abs(err) <= dead) {
            stop_motors();
            return;
        }
        catch_spin_in_place(err < 0);
        return;
    }
    if (abs(err) <= dead) {
        pulse_drive(speed, 0.0f);
        return;
    }
    float om = (abs(err) <= macro) ? CATCH_OMEGA_MICRO : CATCH_OMEGA_MACRO;
    if (err < 0) {
        om = -om;
    }
    pulse_drive(speed, om);
}

bool stage_timeout(void)
{
    return (esp_timer_get_time() - g_stage_t0) > STAGE_TIMEOUT_US;
}

/* ==================== 调试可视化（对齐 cam_line_follow） ==================== */
void catch_debug_copy_images(void)
{
    int w = s_img_w;
    int h = s_img_h;
    for (int y = 0; y < h; y++) {
        uint8_t *dg = s_dbg_gray + y * w;
        uint8_t *db = s_dbg_bin + y * w;
        for (int x = 0; x < w; x++) {
            uint8_t r, g, b;
            rgb_at(x, y, &r, &g, &b);
            dg[x] = (uint8_t)((r * 30 + g * 59 + b * 11) / 100);
            /* 右图：掩码叠加，红/绿/蓝目标高亮 */
            db[x] = 40;
            if (s_mask[y * w + x]) {
                db[x] = 0;
            }
        }
    }
}

void catch_debug_update(const FrameSight *p)
{
    static int dbg_n;
    if ((++dbg_n & 1) != 0) {
        return;
    }
    if (!s_dbg_mutex || !s_dbg_gray || !s_dbg_bin) {
        return;
    }
    if (xSemaphoreTake(s_dbg_mutex, 0) != pdTRUE) {
        return;
    }
    /* 右图掩码跟当前阶段相关：搜/接近球→球色；搜网/对齐/推→绿；回终点→黑 */
    ColorId mask_c = COLOR_RED;
    if (g_state == ST_SEARCH_NET || g_state == ST_ALIGN || g_state == ST_PUSH) {
        mask_c = COLOR_GREEN;
    } else if (g_state == ST_SEARCH_BLACK || g_state == ST_RETURN_END || g_state == ST_BACKUP) {
        mask_c = COLOR_BLACK;
    } else {
        mask_c = (g_ball_kind == BALL_RED) ? COLOR_RED : COLOR_BLUE;
    }
    build_mask(mask_c, (mask_c == COLOR_RED || mask_c == COLOR_BLUE) ? ball_crop_y0() : 0);

    g_dbg_sight = *p;
    g_dbg_state = g_state;
    g_dbg_kind = g_ball_kind;
    g_dbg_w = s_img_w;
    g_dbg_h = s_img_h;
    catch_debug_copy_images();

    int line_dx = (p->ball.found && p->net.found) ? (p->net.cx - p->ball.cx) : 0;
    float d_ball = p->ball.found ? dist_bumper_cm(&p->ball) : -1.0f;
    float d_net = (p->ball.found && p->net.found) ? dist_ball_net_cm(&p->ball, &p->net) : -1.0f;
    g_last_d_ball = d_ball;
    g_last_d_net = d_net;

    int ccx, ccy;
    car_center(&ccx, &ccy);
    int n = snprintf(s_dbg_json, sizeof(s_dbg_json),
                     "{\"st\":\"%s\",\"ball\":\"%s\",\"ms\":%d,\"vy\":%.0f,\"om\":%.0f,"
                     "\"ang\":%.1f,\"ccx\":%d,\"ccy\":%d,\"crop\":%d,"
                     "\"dcm\":%.1f,\"ndcm\":%.1f,\"ldx\":%d,"
                     "\"done\":{\"r\":%d,\"u\":%d,\"n\":%d},"
                     "\"b\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"r\":%d,\"a\":%d,\"c\":%.2f,"
                     "\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"red\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"blu\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"n\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"a\":%d,"
                     "\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d},"
                     "\"blk\":{\"ok\":%d,\"cx\":%d},"
                     "\"lock\":{\"ok\":%d,\"cx\":%d,\"cy\":%d,\"x0\":%d,\"y0\":%d,\"x1\":%d,\"y1\":%d}}",
                     state_name(g_state), ball_name(g_ball_kind), g_decode_ms, g_last_vy, g_last_om,
                     p->align_deg, ccx, ccy, ball_crop_y0(),
                     d_ball, d_net, line_dx,
                     g_done_red ? 1 : 0, g_done_blue ? 1 : 0, balls_done(),
                     p->ball.found ? 1 : 0, p->ball.cx, p->ball.cy, p->ball.radius, p->ball.area,
                     p->ball.circularity, p->ball.x0, p->ball.y0, p->ball.x1, p->ball.y1,
                     p->red.found ? 1 : 0, p->red.cx, p->red.cy, p->red.x0, p->red.y0, p->red.x1, p->red.y1,
                     p->blue.found ? 1 : 0, p->blue.cx, p->blue.cy, p->blue.x0, p->blue.y0, p->blue.x1, p->blue.y1,
                     p->net.found ? 1 : 0, p->net.cx, p->net.cy, p->net.area,
                     p->net.x0, p->net.y0, p->net.x1, p->net.y1,
                     p->has_black_line ? 1 : 0, p->black_cx,
                     g_locked_ball.found ? 1 : 0, g_locked_ball.cx, g_locked_ball.cy,
                     g_locked_ball.x0, g_locked_ball.y0, g_locked_ball.x1, g_locked_ball.y1);
    if (n < 0) {
        n = 0;
    }
    if (n >= (int)sizeof(s_dbg_json)) {
        n = (int)sizeof(s_dbg_json) - 1;
        s_dbg_json[n] = '\0';
    }
    s_dbg_json_len = n;
    s_dbg_ready = true;
    xSemaphoreGive(s_dbg_mutex);
}
/* ==================== 状态机 ==================== */
void mark_ball_done(void)
{
    if (g_ball_kind == BALL_RED) {
        g_done_red = true;
        ESP_LOGI(TAG, "红球已入网  计数 %d/2", balls_done());
    } else {
        g_done_blue = true;
        ESP_LOGI(TAG, "蓝球已入网  计数 %d/2", balls_done());
    }
    memset(&g_locked_ball, 0, sizeof(g_locked_ball));
}

void on_push_success(void)
{
    mark_ball_done();
    stop_motors();
    enter_state(ST_BACKUP);
    g_backup_until = esp_timer_get_time() + (int64_t)BACKUP_MS * 1000;
}

void after_backup(void)
{
    stop_motors();
    if (balls_done() >= 2) {
        ESP_LOGI(TAG, "红蓝都已推入，任务结束");
        enter_state(ST_DONE);
        return;
    }
    ESP_LOGI(TAG, "后退结束，点动找下一颗球");
    enter_state(ST_SEARCH_BALL);
    catch_spin_search_ball(true);
}

void catch_control_once(void)
{
    analyze_frame(&g_sight);
    FrameSight *p = &g_sight;
    g_last_d_ball = p->ball.found ? dist_bumper_cm(&p->ball) : -1.0f;
    g_last_d_net = (p->ball.found && p->net.found) ? dist_ball_net_cm(&p->ball, &p->net) : -1.0f;

    static int64_t last_log = 0;
    int64_t now = esp_timer_get_time();
    if (now - last_log > 300000) {
        ESP_LOGI(TAG,
                 "%s tgt=%s done=%d/2 R%s(%d,%d) U%s(%d,%d) N(%d,%d) "
                 "d=%.1f nd=%.1f vy=%.0f om=%.0f %dms",
                 state_name(g_state), ball_name(g_ball_kind), balls_done(),
                 g_done_red ? "OK" : "",
                 p->red.found ? p->red.cx : -1,
                 p->red.found ? p->red.cy : -1,
                 g_done_blue ? "OK" : "",
                 p->blue.found ? p->blue.cx : -1,
                 p->blue.found ? p->blue.cy : -1,
                 p->net.found ? p->net.cx : -1,
                 p->net.found ? p->net.cy : -1,
                 g_last_d_ball, g_last_d_net, g_last_vy, g_last_om, g_decode_ms);
        last_log = now;
    }

    if (g_state != ST_DONE && g_state != ST_FAIL && g_state != ST_IDLE &&
        g_state != ST_BACKUP && g_state != ST_PUSH &&
        g_state != ST_SEARCH_BALL &&
        stage_timeout()) {
        ESP_LOGW(TAG, "阶段超时，回到搜球 @ %s", state_name(g_state));
        stop_motors();
        enter_state(ST_SEARCH_BALL);
        catch_debug_update(p);
        return;
    }

    switch (g_state) {
    case ST_IDLE:
        stop_motors();
        enter_state(ST_SEARCH_BALL);
        break;

    case ST_SEARCH_BALL:
        if (p->ball.found) {
            stop_motors();
            enter_state(ST_APPROACH_BALL);
        } else {
            catch_spin_search_ball(true);
        }
        break;

    case ST_APPROACH_BALL: {
        if (!p->ball.found) {
            enter_state(ST_SEARCH_BALL);
            catch_spin_in_place(true);
            break;
        }
        float d = dist_bumper_cm(&p->ball);
        int err = p->ball.cx - s_img_w / 2;
        bool x_ok = abs(err) <= center_dead_px() + 2;
        bool low_ok = (p->ball.cy >= s_img_h * 55 / 100) || blob_near_bumper(&p->ball);
        bool at_range = ((d <= APPROACH_STOP_CM + 1.2f) && low_ok) || blob_near_bumper(&p->ball);
        if (at_range && x_ok) {
            stop_motors();
            g_stop_hold++;
            if (g_stop_hold >= STOP_HOLD_FRAMES) {
                ESP_LOGI(TAG, "停在球前 d=%.1fcm cx=%d cy=%d，开始对齐",
                         d, p->ball.cx, p->ball.cy);
                g_locked_ball = p->ball;
                if (ready_to_shoot(p)) {
                    ESP_LOGI(TAG, "连线 %.1f° 已对准且球居中，快速撞球 %.0fms", p->align_deg, (float)PUSH_MS);
                    enter_state(ST_PUSH);
                    catch_drive(PUSH_SPEED, 0.0f);
                } else {
                    enter_state(ST_ALIGN);
                }
            }
        } else {
            g_stop_hold = 0;
            float fwd = at_range ? 0.0f : APPROACH_SPEED;
            pulse_center_on_x(p->ball.cx, fwd);
        }
        break;
    }

    case ST_LOCK_BALL:
    case ST_SEARCH_NET:
        stop_motors();
        enter_state(ST_ALIGN);
        break;

    case ST_ALIGN: {
        if (!p->ball.found) {
            enter_state(ST_SEARCH_BALL);
            catch_spin_in_place(true);
            break;
        }
        if (!p->net.found) {
            catch_spin_in_place(true);
            break;
        }
        int ldx = p->net.cx - p->ball.cx;
        if (ready_to_shoot(p)) {
            stop_motors();
            g_lock_frames++;
            g_orbit_dir_valid = false;
            if (g_lock_frames >= ALIGN_HOLD_FRAMES) {
                ESP_LOGI(TAG, "连线 %.1f° ldx=%d 球居中，快速撞球 %.0fms",
                         p->align_deg, ldx, (float)PUSH_MS);
                enter_state(ST_PUSH);
                catch_drive(PUSH_SPEED, 0.0f);
            }
        } else {
            g_lock_frames = 0;
            if (line_almost_vertical(p) && !ball_on_center(p)) {
                pulse_center_on_x(p->ball.cx, 0.0f);
            } else {
                pulse_orbit_around_front(align_orbit_left(ldx));
            }
        }
        break;
    }

    case ST_PUSH: {
        if (esp_timer_get_time() - g_stage_t0 >= (int64_t)PUSH_MS * 1000) {
            ESP_LOGI(TAG, "撞击 %.0fms 结束，匀速倒退 %.0fms", (float)PUSH_MS, (float)BACKUP_MS);
            on_push_success();
        } else {
            catch_drive(PUSH_SPEED, 0.0f);
        }
        break;
    }

    case ST_BACKUP:
        if (esp_timer_get_time() >= g_backup_until) {
            after_backup();
        } else {
            catch_drive(-BACKUP_SPEED, 0.0f);
        }
        break;

    case ST_SEARCH_BLACK:
    case ST_RETURN_END:
        enter_state(ST_SEARCH_BALL);
        catch_spin_in_place(true);
        break;

    case ST_DONE:
        stop_motors();
        g_mission = MISSION_DONE;
        break;

    case ST_FAIL:
        stop_motors();
        ESP_LOGW(TAG, "从 FAIL 恢复，重新搜球");
        enter_state(ST_SEARCH_BALL);
        catch_spin_in_place(true);
        break;

    default:
        enter_state(ST_FAIL);
        break;
    }

    catch_debug_update(p);
}
