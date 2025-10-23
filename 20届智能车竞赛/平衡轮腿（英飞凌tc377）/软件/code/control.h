#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "kalman_rm.h"
#include"engine.h"
#include "small_driver_uart_control.h"
#include "jump_control.h"
#include "image.h"
#include"leg_adaptive.h"

extern float target_velocity;//Ŀ���ٶ�ֵ0��������400
extern float now_velocity;//ʵ���ٶ�ֵ
extern float Encoder_Left, Encoder_Right; // ���ҵ��������ֵ
extern float target_motor_Stand;//��е��ֵ
extern float target_engine_high;//�Ȳ��߶���Ϣ
extern float target_motor_angle;//����Ƕ���Ϣ
extern float angel_init;//����Ƕȳ�ʼ����Ϣ
extern float Turn_Pwm; // ת��PWMֵ
extern int stop_flash;//������־λ
extern int bridge_high;//�����Ÿߵͱ�־λ
extern int speed_up;//���ٱ�־λ
extern pid_param_t motor_direction; // ����PID����
#define PRECISION 0.001
#define MAX_DUTY                (65)
void Balance_init(void);
void balance_control(void);
void leg_control(float *x, float *y) ;
float leg_velocity(float leg_Kp,int Velocity_Pwm);
void adjust_pid_based_on_leg_height(float *current_leg_height);
bool is_airborne();
void pid_high_init() ;
#endif /* CODE_ENGINE_H_ */
