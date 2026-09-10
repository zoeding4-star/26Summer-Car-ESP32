#pragma once

#include <stdbool.h>
#include <stdint.h>

uint8_t luma_at(int x, int y);
void rgb_at(int x, int y, uint8_t *r, uint8_t *g, uint8_t *b);
bool decode_mjpeg(const uint8_t *jpg, int len);
bool decode_mjpeg_rgb(const uint8_t *jpg, int len);
