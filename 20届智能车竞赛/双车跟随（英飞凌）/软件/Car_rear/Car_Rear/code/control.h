#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "zf_common_headfile.h"

typedef enum
{
    left_motor,right_motor
}motor_enum;


#define SERVO_MOTOR_PWM             (ATOM2_CH7_P11_12)                           // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (300)                                       // 定义主板上舵机频率  请务必注意范围 50-300
#define STEER_DUTY_MIDDLE           (4992)
#define STEER_DUTY_LEFT             (5424)
#define STEER_DUTY_RIGHT            (4560)

#define DIR_R                       (P33_1)
#define PWM_R                       (ATOM2_CH1_P33_5)
#define DIR_L                       (P33_0)
#define PWM_L                       (ATOM2_CH0_P33_4)

#define ENCODER_L                   (TIM4_ENCODER)                         // 带方向编码器对应使用的编码器接口
#define ENCODER_L_PULSE             (TIM4_ENCODER_CH1_P02_8)               // PULSE 对应的引脚
#define ENCODER_L_DIR               (TIM4_ENCODER_CH2_P00_9)               // DIR 对应的引脚
#define ENCODER_R                   (TIM2_ENCODER)                         // 带方向编码器对应使用的编码器接口
#define ENCODER_R_PULSE             (TIM2_ENCODER_CH1_P33_7)               // PULSE 对应的引脚
#define ENCODER_R_DIR               (TIM2_ENCODER_CH2_P33_6)               // DIR 对应的引脚
#define ENCODER_FILTER_SIZE         (20)                                   // 均值滤波长度

#define BLDC_L_PIN                  (ATOM3_CH5_P11_10)
#define BLDC_R_PIN                  (ATOM3_CH3_P11_6)


#define START_DELAY_TIME            (200)                                   // 启动延时时间（START_DELAY_TIME*10ms）


//无刷电机开关函数（0关100满）
#define BLDC_L(x)                   (pwm_set_duty(BLDC_L_PIN,5*(x)+500))
#define BLDC_R(x)                   (pwm_set_duty(BLDC_R_PIN,5*(x)+500))

void Steer_Init(void);
void Steer_Control(int16 duty);
void Motor_Init(void);
void Motor_Control(motor_enum num,int16 duty);
void Encoder_Init(void);
void Bldc_Init(void);
void Speed_Loop(void);
void Dir_Loop(void);
void Target_Speed_Loop(void);
void Run_State_Update(void);


extern int16 encoder_data_L,encoder_data_R;    //编码器读数
extern int16 encoder_data_window_L[ENCODER_FILTER_SIZE],encoder_data_window_R[ENCODER_FILTER_SIZE];    //均值滤波数组
extern float LP_filter_coef; //低通滤波系数
extern float speed_L,speed_R;  //速度
extern float target_speed_L,target_speed_R;    //目标速度
extern float target_speed,target_speed_set; //目标速度
extern float target_speed_offset;  //差速
extern float target_speed_offset_coef; //差速系数
extern uint8 start_flag;     //启动标志位
extern uint8 start_delay_count;   //启动延时计数
extern uint8 start_delay_count_flag;
extern uint8 run_state;
extern float target_distance_coef;
extern float dir_p_set;
extern float dynamic_p_coef;


#endif
