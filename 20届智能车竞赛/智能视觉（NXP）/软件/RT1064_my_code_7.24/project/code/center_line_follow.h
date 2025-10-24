#ifndef _CENTER_LINE_FOLLOW_H_
#define _CENTER_LINE_FOLLOW_H_

extern pid_controller cl_angle_pid;//center line angle pid
extern pid_controller dv_pid;
extern pid_controller cl_angle_pid_mini;
extern pid_controller mcx_pid;

float slope_to_angle(float slope);
void centerline_follow(uint8 *center_line, uint8* move_angle, int16* turn_angle);
void move_ctrl(int move_speed, uint8 move_angle, int16 turn_angle);
#endif
