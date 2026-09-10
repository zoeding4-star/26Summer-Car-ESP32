#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "cam_types.h"

void rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b, int *h, int *s, int *v);
bool pixel_is_color(ColorId id, uint8_t r, uint8_t g, uint8_t b, int h, int s, int v);
void build_mask(ColorId id, int y0);
bool flood_blob(int sx, int sy, BlobTarget *out);
void dilate_mask(int times);
bool mask_as_blob(int min_a, BlobTarget *out);
int ball_crop_y0(void);
bool find_best_blob(ColorId id, bool need_round, int min_a, int max_a, BlobTarget *best);
bool pick_net_among(const BlobTarget *cands, int n, const BlobTarget *ball, BlobTarget *best);
bool find_net_for_ball(const BlobTarget *ball, BlobTarget *best);
bool detect_black_line(int *cx_out);
void car_center(int *cx, int *cy);
float align_angle_deg(const BlobTarget *ball, const BlobTarget *net);
bool line_almost_vertical(const FrameSight *p);
int balls_done(void);
void pick_search_target(FrameSight *out);
void analyze_frame(FrameSight *out);
bool blob_near_bumper(const BlobTarget *b);
float px_per_cm(void);
float dist_bumper_cm(const BlobTarget *b);
float dist_ball_net_cm(const BlobTarget *b, const BlobTarget *n);
int center_dead_px(void);
int center_macro_px(void);
bool ball_on_center(const FrameSight *p);
