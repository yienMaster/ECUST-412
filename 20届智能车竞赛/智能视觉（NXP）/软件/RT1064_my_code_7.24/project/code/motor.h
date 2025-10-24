#ifndef _MOTOR_H_
#define _MOTOR_H_

extern pid_controller motor1_pid;
extern pid_controller motor2_pid;
extern pid_controller motor3_pid;
void motor_init();
void set_motor1_vel(int vel);
void set_motor2_vel(int vel);
void set_motor3_vel(int vel);


#endif
