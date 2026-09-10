#include "util.h"
#include "cam_config.h"
#include "cam_state.h"
#include "esp_heap_caps.h"
#include <math.h>

float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
void *psram_alloc(size_t n)
{
    void *p = heap_caps_aligned_alloc(16, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_aligned_alloc(16, n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

int map_x(int x, int w)
{
    if (x < 0) x = 0;
    if (x >= w) x = w - 1;
    return CAM_FLIP_LR ? (w - 1 - x) : x;
}

int map_y(int y, int h)
{
    if (y < 0) y = 0;
    if (y >= h) y = h - 1;
    return CAM_FLIP_UD ? (h - 1 - y) : y;
}
int scaled_px(int px480)
{
    int v = px480 * s_img_w / CAM_WIDTH;
    return (v < 1) ? 1 : v;
}
