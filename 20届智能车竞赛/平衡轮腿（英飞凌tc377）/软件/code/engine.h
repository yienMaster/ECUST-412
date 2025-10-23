#ifndef CODE_ENGINE_H_
#define CODE_ENGINE_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include"control.h"

//#define PWM_1              (ATOM0_CH4_P02_4)//��2 900���� +
//#define PWM_2              (ATOM2_CH3_P33_7)//��1 900���� +
//#define PWM_3              (ATOM3_CH2_P33_6)//��1 900���� -
//#define PWM_4              (ATOM0_CH5_P02_5)//��2 900���� -
#define PWM_1              (ATOM2_CH5_P33_13)//��2 900���� +
#define PWM_2              (ATOM2_CH3_P33_7)//��1 900���� +
#define PWM_3              (ATOM3_CH2_P33_6)//��1 900���� -
#define PWM_4              (ATOM2_CH4_P33_8)//��2 900���� -
#define FREQ               (50)

//��1       ��1
//   ����
//   364
//��2       ��2

//��ʼ��ʹ��
void engine_init(int pwm1,int pwm2);
uint32 auu(uint32 c);

//��ѭ��ʹ��
void engine_maintain(int pwm1,int pwm2);
void engine_left_maintain(int pwm1,int pwm2);
void engine_right_maintain(int pwm1,int pwm2);
void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid1, pid_param_t * pid2);

//һ���Ե���ʹ��
void engine_jump(void);

#endif /* CODE_ENGINE_H_ */
