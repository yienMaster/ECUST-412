#ifndef _MY_PIT_H_
#define _MY_PIT_H_
extern bool pit_flag;
extern bool speak;
extern uint8 m_angle;
extern int16 t_angle;

extern int16 cnt_01s;
extern int16 cnt_1ms;
extern int my_move_dir;
extern uint8 mcx_find_flag;
extern int mcx_info;
extern uint8 mcx_detect_flag;
extern uint8 mcx_temp;
extern uint8 mcx_lorr;
extern uint8 l_or_r;

void pit_handler();
	

#endif
