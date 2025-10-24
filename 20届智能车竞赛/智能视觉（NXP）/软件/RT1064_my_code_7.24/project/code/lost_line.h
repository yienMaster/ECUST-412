#ifndef _LOST_LINE_H_
#define _LOST_LINE_H_

//丢线标志位
extern uint8 left_down_lost;
extern uint8 right_down_lost;
extern uint8 left_up_lost;
extern uint8 right_up_lost;

extern uint8 left_down_y;
extern uint8 right_down_y;
extern uint8 left_up_y;
extern uint8 right_up_y;

extern uint8 down_reach_left;
extern uint8 down_reach_right;


void lost_line(uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r, uint16(*points_l)[2], uint16(*points_r)[2]);
float arr_average(uint16* arr, uint8 _start, uint8 _end);
uint16 find_v_point(uint8(*image)[image_w], uint16 total_num, uint16(*points)[2]);
void fill_line(uint8 *l, uint8 *r, unsigned char(*p)[image_w], unsigned short *indexl, unsigned short *indexr);

#endif

