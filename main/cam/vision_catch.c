#include "vision_catch.h"
#include "cam_config.h"
#include "cam_state.h"
#include "decode.h"
#include "util.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, int *h, int *s, int *v)
{
    int maxc = r;
    if (g > maxc) maxc = g;
    if (b > maxc) maxc = b;
    int minc = r;
    if (g < minc) minc = g;
    if (b < minc) minc = b;
    int delta = maxc - minc;

    *v = maxc;
    if (maxc == 0) {
        *s = 0;
        *h = 0;
        return;
    }
    *s = delta * 255 / maxc;
    if (delta == 0) {
        *h = 0;
        return;
    }

    int hh;
    if (maxc == r) {
        hh = 60 * (g - b) / delta;
    } else if (maxc == g) {
        hh = 120 + 60 * (b - r) / delta;
    } else {
        hh = 240 + 60 * (r - g) / delta;
    }
    if (hh < 0) {
        hh += 360;
    }
    *h = hh / 2;    /* OpenCV 风格 0~179 */
}

/*
 * 颜色策略（像素级 HSV + RGB 主导色，块级再滤一次）：
 * 红球：色相两端 + R 明显大于 G/B，避免木头/橙色干扰。
 * 蓝球：放宽暗度(V≥18,S≥28)，色相约 82–155，且 B 主导——深蓝色球
 *       在相机里往往很暗，旧门槛 V≥40 会整块丢掉；灰/黑没有 B>R、B>G。
 * 荧光绿网：高饱和高亮 + 黄绿到绿的色相 + G 远大于 R/B。
 * 黑杆排除：杆上的绿反光通常又暗又细长；块级丢掉低亮度、瘦高竖条，
 *           并按 area*mean_v 选“又大又亮”的网，而不是最大的暗斑。
 */
bool pixel_is_color(ColorId id, uint8_t r, uint8_t g, uint8_t b, int h, int s, int v)
{
    switch (id) {
    case COLOR_RED:
        if (!((s >= 55 && v >= 40) && (h <= 14 || h >= 165))) {
            return false;
        }
        return (r >= g + 18) && (r >= b + 18);
    case COLOR_GREEN:
        /* CAM_TUNE 绿网默认：H 35–95 S≥45 V≥55，G 主导 */
        if (!((s >= 45 && v >= 55) && (h >= 35 && h <= 95))) {
            return false;
        }
        return (g >= 40) && (g >= r + 18) && (g >= b + 12);
    case COLOR_BLUE:
        /* CAM_TUNE 蓝球默认：H 85–150 S≥20 V≥12，B 主导 */
        if (!((s >= 20 && v >= 12) && (h >= 85 && h <= 150))) {
            return false;
        }
        return (b >= 30) && (b >= r + 16) && (b >= g + 10);
    case COLOR_BLACK:
        return (v <= 55) && (s <= 90);
    default:
        return false;
    }
}

void build_mask(ColorId id, int y0)
{
    int n = s_img_w * s_img_h;
    memset(s_mask, 0, (size_t)n);
    memset(s_visited, 0, (size_t)n);
    if (y0 < 0) {
        y0 = 0;
    }
    for (int y = y0; y < s_img_h; y++) {
        if ((y & 15) == 0) {
            vTaskDelay(0);
        }
        for (int x = 0; x < s_img_w; x++) {
            uint8_t r, g, b;
            rgb_at(x, y, &r, &g, &b);
            int hh, ss, vv;
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            if (pixel_is_color(id, r, g, b, hh, ss, vv)) {
                s_mask[y * s_img_w + x] = 1;
            }
        }
    }
}

bool flood_blob(int sx, int sy, BlobTarget *out)
{
    static Pt16 stack[FLOOD_STACK];
    int sp = 0;
    int w = s_img_w;
    int h = s_img_h;
    size_t idx0 = (size_t)sy * w + sx;
    if (!s_mask[idx0] || s_visited[idx0]) {
        return false;
    }

    int minx = sx, maxx = sx, miny = sy, maxy = sy;
    int64_t sumx = 0, sumy = 0;
    int64_t sumr = 0, sumg = 0, sumb = 0, sumv = 0;
    int area = 0;
    int peri = 0;

    stack[sp++] = (Pt16){ (int16_t)sx, (int16_t)sy };
    s_visited[idx0] = 1;

    while (sp > 0) {
        Pt16 p = stack[--sp];
        area++;
        sumx += p.x;
        sumy += p.y;
        {
            uint8_t r, g, b;
            int hh, ss, vv;
            rgb_at(p.x, p.y, &r, &g, &b);
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            sumr += r;
            sumg += g;
            sumb += b;
            sumv += vv;
        }
        if (p.x < minx) minx = p.x;
        if (p.x > maxx) maxx = p.x;
        if (p.y < miny) miny = p.y;
        if (p.y > maxy) maxy = p.y;

        static const int dx[4] = {1, -1, 0, 0};
        static const int dy[4] = {0, 0, 1, -1};
        for (int i = 0; i < 4; i++) {
            int nx = p.x + dx[i];
            int ny = p.y + dy[i];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                peri++;
                continue;
            }
            size_t ni = (size_t)ny * w + nx;
            if (!s_mask[ni]) {
                peri++;
                continue;
            }
            if (s_visited[ni]) {
                continue;
            }
            if (sp >= FLOOD_STACK) {
                continue;
            }
            s_visited[ni] = 1;
            stack[sp++] = (Pt16){ (int16_t)nx, (int16_t)ny };
        }
    }

    if (area < 4) {
        return false;
    }

    float circ = 0.0f;
    if (peri > 0) {
        circ = (4.0f * 3.1415926f * (float)area) / ((float)peri * (float)peri);
    }

    out->found = true;
    out->area = area;
    out->cx = (int)(sumx / area);
    out->cy = (int)(sumy / area);
    out->x0 = minx;
    out->y0 = miny;
    out->x1 = maxx;
    out->y1 = maxy;
    out->radius = (int)(0.5f * sqrtf((float)area / 3.1415926f) + 0.5f);
    out->circularity = circ;
    out->mr = (uint8_t)(sumr / area);
    out->mg = (uint8_t)(sumg / area);
    out->mb = (uint8_t)(sumb / area);
    out->mean_v = (int)(sumv / area);
    return true;
}

void dilate_mask(int times)
{
    int w = s_img_w;
    int h = s_img_h;
    int np = w * h;
    for (int k = 0; k < times; k++) {
        memcpy(s_visited, s_mask, (size_t)np);
        for (int y = 0; y < h; y++) {
            if ((y & 7) == 0) {
                vTaskDelay(0);
            }
            for (int x = 0; x < w; x++) {
                size_t i = (size_t)y * w + x;
                if (s_visited[i]) {
                    s_mask[i] = 1;
                    continue;
                }
                int hit = 0;
                if (x > 0 && s_visited[i - 1]) {
                    hit = 1;
                }
                if (x + 1 < w && s_visited[i + 1]) {
                    hit = 1;
                }
                if (y > 0 && s_visited[i - (size_t)w]) {
                    hit = 1;
                }
                if (y + 1 < h && s_visited[i + (size_t)w]) {
                    hit = 1;
                }
                if (hit) {
                    s_mask[i] = 1;
                }
            }
        }
    }
    memset(s_visited, 0, (size_t)np);
}

bool mask_as_blob(int min_a, BlobTarget *out)
{
    memset(out, 0, sizeof(*out));
    int64_t sumx = 0, sumy = 0, sumr = 0, sumg = 0, sumb = 0, sumv = 0;
    int area = 0;
    int minx = s_img_w, maxx = 0, miny = s_img_h, maxy = 0;
    for (int y = 0; y < s_img_h; y++) {
        for (int x = 0; x < s_img_w; x++) {
            if (!s_mask[y * s_img_w + x]) {
                continue;
            }
            area++;
            sumx += x;
            sumy += y;
            if (x < minx) minx = x;
            if (x > maxx) maxx = x;
            if (y < miny) miny = y;
            if (y > maxy) maxy = y;
            uint8_t r, g, b;
            int hh, ss, vv;
            rgb_at(x, y, &r, &g, &b);
            rgb_to_hsv(r, g, b, &hh, &ss, &vv);
            sumr += r;
            sumg += g;
            sumb += b;
            sumv += vv;
        }
    }
    if (area < min_a) {
        return false;
    }
    out->found = true;
    out->area = area;
    out->cx = (int)(sumx / area);
    out->cy = (int)(sumy / area);
    out->x0 = minx;
    out->y0 = miny;
    out->x1 = maxx;
    out->y1 = maxy;
    out->radius = (int)(0.5f * sqrtf((float)area / 3.1415926f) + 0.5f);
    out->circularity = 0.0f;
    out->mr = (uint8_t)(sumr / area);
    out->mg = (uint8_t)(sumg / area);
    out->mb = (uint8_t)(sumb / area);
    out->mean_v = (int)(sumv / area);
    return true;
}

int ball_crop_y0(void)
{
    return s_img_h * BALL_FAR_CROP_NUM / BALL_FAR_CROP_DEN;
}

bool find_best_blob(ColorId id, bool need_round, int min_a, int max_a, BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    int y0 = need_round ? ball_crop_y0() : 0;
    build_mask(id, y0);
    if (!need_round) {
        dilate_mask(1);
    }

    BlobTarget cand;
    BlobTarget largest_any;
    memset(&largest_any, 0, sizeof(largest_any));
    int n_ok = 0;
    int best_score = -1;
    float circ_min = MIN_CIRCULARITY;
    float ar_min = 0.55f;
    if (need_round && id == COLOR_BLUE) {
        circ_min = MIN_CIRCULARITY_BLUE;
        ar_min = 0.45f;
    }
    for (int y = y0; y < s_img_h; y++) {
        for (int x = 0; x < s_img_w; x++) {
            size_t i = (size_t)y * s_img_w + x;
            if (!s_mask[i] || s_visited[i]) {
                continue;
            }
            memset(&cand, 0, sizeof(cand));
            if (!flood_blob(x, y, &cand)) {
                continue;
            }
            if (!largest_any.found || cand.area > largest_any.area) {
                largest_any = cand;
            }
            if (cand.area < min_a || cand.area > max_a) {
                continue;
            }
            if (need_round && cand.circularity < circ_min) {
                continue;
            }
            int bw = cand.x1 - cand.x0 + 1;
            int bh = cand.y1 - cand.y0 + 1;
            float ar = (bw < bh) ? (float)bw / (float)bh : (float)bh / (float)bw;
            if (need_round && ar < ar_min) {
                continue;
            }
            if (need_round && id == COLOR_RED) {
                if (!(cand.mr >= cand.mg + 12 && cand.mr >= cand.mb + 12)) {
                    continue;
                }
            }
            if (need_round && id == COLOR_BLUE) {
                if (!(cand.mb >= cand.mr + 8 && cand.mb >= cand.mg + 5)) {
                    continue;
                }
            }
            if (!need_round) {
                if (ar < 0.22f && bw < (s_img_w / 8 + 1)) {
                    continue;
                }
                if (cand.mean_v < 50) {
                    continue;
                }
                if (bh * 10 >= bw * 20 && bw < (s_img_w * 17 / 100 + 1)) {
                    continue;
                }
                if (!(cand.mg >= cand.mr + 12 && cand.mg >= cand.mb + 8)) {
                    continue;
                }
            }
            if (need_round && id == COLOR_BLUE) {
                if (bh * 10 >= bw * 18 && bw < (s_img_w * 17 / 100 + 1)) {
                    continue;
                }
            }
            int score = need_round ? cand.area : (cand.area * (cand.mean_v + 1));
            n_ok++;
            if (!best->found || score > best_score) {
                *best = cand;
                best_score = score;
            }
            if (n_ok >= MAX_CC_BLOBS) {
                goto done;
            }
        }
    }
done:
    if (!best->found && !need_round) {
        if (largest_any.found && largest_any.area >= 8) {
            int bw = largest_any.x1 - largest_any.x0 + 1;
            int bh = largest_any.y1 - largest_any.y0 + 1;
            bool pole = (bh * 10 >= bw * 20) && (bw < s_img_w / 8 + 1);
            if (!pole) {
                *best = largest_any;
            }
        }
        if (!best->found) {
            mask_as_blob(8, best);
        }
    }
    return best->found;
}

/* 在已通过绿网形状过滤的块里，选和球同一侧且 |cx| 最近的门 */
bool pick_net_among(const BlobTarget *cands, int n, const BlobTarget *ball,
                           BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    if (n <= 0) {
        return false;
    }
    if (!ball || !ball->found) {
        int best_score = -1;
        for (int i = 0; i < n; i++) {
            int score = cands[i].area * (cands[i].mean_v + 1);
            if (!best->found || score > best_score) {
                *best = cands[i];
                best_score = score;
            }
        }
        return best->found;
    }

    int mid = s_img_w / 2;
    bool prefer_left = (ball->cx < mid);
    int best_i = -1;
    int best_dx = 99999;
    int best_cy = 99999;

    for (int pass = 0; pass < 2 && best_i < 0; pass++) {
        for (int i = 0; i < n; i++) {
            bool same_half = prefer_left ? (cands[i].cx < mid) : (cands[i].cx >= mid);
            if (pass == 0 && !same_half) {
                continue;
            }
            int dx = abs(cands[i].cx - ball->cx);
            if (dx < best_dx || (dx == best_dx && cands[i].cy < best_cy)) {
                best_dx = dx;
                best_cy = cands[i].cy;
                best_i = i;
            }
        }
    }
    if (best_i < 0) {
        return false;
    }
    *best = cands[best_i];
    return true;
}

bool find_net_for_ball(const BlobTarget *ball, BlobTarget *best)
{
    memset(best, 0, sizeof(*best));
    build_mask(COLOR_GREEN, 0);
    dilate_mask(1);

    BlobTarget cands[MAX_CC_BLOBS];
    int n = 0;
    for (int y = 0; y < s_img_h; y++) {
        for (int x = 0; x < s_img_w; x++) {
            size_t i = (size_t)y * s_img_w + x;
            if (!s_mask[i] || s_visited[i]) {
                continue;
            }
            BlobTarget cand;
            memset(&cand, 0, sizeof(cand));
            if (!flood_blob(x, y, &cand)) {
                continue;
            }
            if (cand.area < MIN_NET_AREA || cand.area > MAX_NET_AREA) {
                continue;
            }
            int bw = cand.x1 - cand.x0 + 1;
            int bh = cand.y1 - cand.y0 + 1;
            float ar = (bw < bh) ? (float)bw / (float)bh : (float)bh / (float)bw;
            if (ar < 0.22f && bw < (s_img_w / 8 + 1)) {
                continue;
            }
            if (cand.mean_v < 50) {
                continue;
            }
            if (bh * 10 >= bw * 20 && bw < (s_img_w * 17 / 100 + 1)) {
                continue;
            }
            if (!(cand.mg >= cand.mr + 12 && cand.mg >= cand.mb + 8)) {
                continue;
            }
            if (n < MAX_CC_BLOBS) {
                cands[n++] = cand;
            }
            if (n >= MAX_CC_BLOBS) {
                goto picked;
            }
        }
    }
picked:
    return pick_net_among(cands, n, ball, best);
}

bool detect_black_line(int *cx_out)
{
    build_mask(COLOR_BLACK, 0);
    int y0 = s_img_h * 55 / 100;
    int y1 = s_img_h - 1;
    int total = 0;
    int black = 0;
    int64_t sumx = 0;
    for (int y = y0; y <= y1; y++) {
        for (int x = 0; x < s_img_w; x++) {
            total++;
            if (s_mask[y * s_img_w + x]) {
                black++;
                sumx += x;
            }
        }
    }
    if (total <= 0) {
        return false;
    }
    int pct = black * 100 / total;
    if (pct < BLACK_LINE_MIN_PCT) {
        return false;
    }
    if (cx_out) {
        *cx_out = (black > 0) ? (int)(sumx / black) : s_img_w / 2;
    }
    return true;
}

/* 车心取画面底部中心（近端） */
void car_center(int *cx, int *cy)
{
    *cx = s_img_w / 2;
    *cy = s_img_h - 2;
}

/* 球→网连线相对竖直的夹角：正=网在右侧 */
float align_angle_deg(const BlobTarget *ball, const BlobTarget *net)
{
    float dx = (float)(net->cx - ball->cx);
    float dy = (float)(ball->cy - net->cy); /* 画面向上为正 */
    if (fabsf(dx) < 0.5f && fabsf(dy) < 0.5f) {
        return 0.0f;
    }
    return atan2f(dx, dy) * 180.0f / 3.1415926f;
}

bool line_almost_vertical(const FrameSight *p)
{
    if (!p || !p->ball.found || !p->net.found) {
        return false;
    }
    int ldx = p->net.cx - p->ball.cx;
    return (fabsf(p->align_deg) <= ALIGN_OK_DEG) && (abs(ldx) <= ALIGN_VERT_PX);
}

int balls_done(void)
{
    return (g_done_red ? 1 : 0) + (g_done_blue ? 1 : 0);
}

void pick_search_target(FrameSight *out)
{
    bool take_red = !g_done_red && out->red.found;
    bool take_blue = !g_done_blue && out->blue.found;
    if (take_red && take_blue) {
        if (out->red.cy >= out->blue.cy) {
            g_ball_kind = BALL_RED;
            out->ball = out->red;
        } else {
            g_ball_kind = BALL_BLUE;
            out->ball = out->blue;
        }
        return;
    }
    if (take_red) {
        g_ball_kind = BALL_RED;
        out->ball = out->red;
        return;
    }
    if (take_blue) {
        g_ball_kind = BALL_BLUE;
        out->ball = out->blue;
        return;
    }
    memset(&out->ball, 0, sizeof(out->ball));
}

void analyze_frame(FrameSight *out)
{
    memset(out, 0, sizeof(*out));
    find_best_blob(COLOR_RED, true, MIN_BALL_AREA, MAX_BALL_AREA, &out->red);
    vTaskDelay(0);
    find_best_blob(COLOR_BLUE, true, MIN_BALL_AREA, MAX_BALL_AREA, &out->blue);
    vTaskDelay(0);

    if (g_state == ST_SEARCH_BALL || g_state == ST_IDLE) {
        pick_search_target(out);
    } else if (g_ball_kind == BALL_RED) {
        out->ball = out->red;
    } else {
        out->ball = out->blue;
    }

    find_net_for_ball(out->ball.found ? &out->ball : NULL, &out->net);
    if (g_state == ST_SEARCH_BLACK || g_state == ST_RETURN_END) {
        out->has_black_line = detect_black_line(&out->black_cx);
    }

    if (out->ball.found && out->net.found) {
        out->align_deg = align_angle_deg(&out->ball, &out->net);
    }
}

bool blob_near_bumper(const BlobTarget *b)
{
    if (!b || !b->found) {
        return false;
    }
    int y_lim = s_img_h * NEAR_Y_PCT / 100;
    return (b->cy >= y_lim) || (b->y1 >= s_img_h * 82 / 100);
}

/* 近处约 20cm 地面映在画面下半，用来把像素换成厘米 */
float px_per_cm(void)
{
    if (s_img_h < 8) {
        return 2.0f;
    }
    return (float)s_img_h * 0.50f / 20.0f;
}

float dist_bumper_cm(const BlobTarget *b)
{
    if (!b || !b->found || s_img_h <= 1) {
        return 99.0f;
    }
    int from_bottom = s_img_h - 1 - b->cy;
    if (from_bottom < 0) {
        from_bottom = 0;
    }
    return (float)from_bottom / px_per_cm();
}

float dist_ball_net_cm(const BlobTarget *b, const BlobTarget *n)
{
    if (!b || !n || !b->found || !n->found) {
        return 99.0f;
    }
    float dx = (float)(b->cx - n->cx);
    float dy = (float)(b->cy - n->cy);
    return sqrtf(dx * dx + dy * dy) / px_per_cm();
}

int center_dead_px(void)
{
    return scaled_px(CENTER_DEAD_PX * CAM_WIDTH / 60);
}

int center_macro_px(void)
{
    return scaled_px(CENTER_MACRO_PX * CAM_WIDTH / 60);
}

bool ball_on_center(const FrameSight *p)
{
    if (!p || !p->ball.found) {
        return false;
    }
    return abs(p->ball.cx - s_img_w / 2) <= center_dead_px() + 2;
}
