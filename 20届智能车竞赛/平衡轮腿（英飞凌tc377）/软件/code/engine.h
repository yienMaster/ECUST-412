#ifndef CODE_ENGINE_H_
#define CODE_ENGINE_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include"control.h"

//#define PWM_1              (ATOM0_CH4_P02_4)//左2 900向上 +
//#define PWM_2              (ATOM2_CH3_P33_7)//左1 900向下 +
//#define PWM_3              (ATOM3_CH2_P33_6)//右1 900向上 -
//#define PWM_4              (ATOM0_CH5_P02_5)//右2 900向下 -
#define PWM_1              (ATOM2_CH5_P33_13)//左2 900向上 +
#define PWM_2              (ATOM2_CH3_P33_7)//左1 900向下 +
#define PWM_3              (ATOM3_CH2_P33_6)//右1 900向上 -
#define PWM_4              (ATOM2_CH4_P33_8)//右2 900向下 -
#define FREQ               (50)

//左1       右1
//   九轴
//   364
//左2       右2

//初始化使用
void engine_init(int pwm1,int pwm2);
uint32 auu(uint32 c);

//主循环使用
void engine_maintain(int pwm1,int pwm2);
void engine_left_maintain(int pwm1,int pwm2);
void engine_right_maintain(int pwm1,int pwm2);
void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid1, pid_param_t * pid2);

//一次性单独使用
void engine_jump(void);

#endif /* CODE_ENGINE_H_ */
