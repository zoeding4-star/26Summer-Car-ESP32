#pragma once

#include <stddef.h>

float clampf(float v, float lo, float hi);
void *psram_alloc(size_t n);
int map_x(int x, int w);
int map_y(int y, int h);
int scaled_px(int px480);
