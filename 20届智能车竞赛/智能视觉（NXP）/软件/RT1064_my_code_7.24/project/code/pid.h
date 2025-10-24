#ifndef _PID_H_
#define _PID_H_

#define MAX_ALLOWED_CHANGE 800

typedef struct {
    float Kp;               // ????
    float Ki;               // ????
		float Kp_high;
    float Kd;               // ????
    float e_prev;           // ??????
    float e_prev_prev;      // ??????
    float output_prev;       // ???????
    float output_max;        // ???????
    float output_min;        // ???????
		float right;
		float d_prev;
		float pwm;
		float prev_pwm;
		float prev_vel;
		float adapt_pid_range;
} pid_controller;
float PID_control(float target, float current, pid_controller* pid);
void set_pid(float kp, float ki, float kd, pid_controller* pid);
int pid_output(int target_vel, int filted_edata, pid_controller *pid);
#endif
