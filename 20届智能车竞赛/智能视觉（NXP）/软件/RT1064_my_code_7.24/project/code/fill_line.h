#ifndef _FILL_LINE_H_
#define _FILL_LINE_H_
#include "image.h"
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint16 total_num_l, uint16 total_num_r, uint8 *r_border, uint8 state);
uint8 round_judge(void);
void road_about_dispose(void);
uint8 line_straight(uint8* arr);

#endif
