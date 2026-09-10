#pragma once

#include <stdbool.h>
#include "cam_types.h"

int roi_x0(void);
int roi_x1(void);
int max_line_w(void);
int pick_stem_cx(const RowScan *row, int pred, int max_jump);
void probe_sides(int cx, int y, int *left, int *right);
const char *view_name(ViewType t);
Sight look(void);
void follow_stem(const Sight *p);
bool heading_ok(const Sight *p);
void begin_phase(Phase ph);
void align_heading(const Sight *p);
void follow_tick(const Sight *p);
void control_once(void);
