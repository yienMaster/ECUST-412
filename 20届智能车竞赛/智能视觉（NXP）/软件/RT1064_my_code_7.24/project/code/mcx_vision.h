#ifndef _MCX_VISION_H_
#define _MCX_VISION_H_

extern char mcx_data[50];
extern int16 mcx_turn_angle;
extern int16 mcx_move_speed;
extern int mcx_cnt_01s;
extern int mcx_flag;
extern int d_count;
extern int mcx_pid_count;//Í£³µpid¿ØÖÆ
extern int mcx_time_flag;
extern int mcx_time;
void mcx_info_get(void);
void box_detect(int mcx_dat);
void box_pull_right (void);
void box_pull_left (void);
void box_pull(int l_or_r);
extern uint8 ort_transport;
#endif
