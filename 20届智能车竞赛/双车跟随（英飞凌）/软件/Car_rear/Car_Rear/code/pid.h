#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"

//PID结构体
typedef struct{
        float p;
        float i;
        float d;
        float error0;
        float error00;
        float out;
        float i_max;
        float out_max;
        float out_min;
        float integrator;
}PID_Structure;


#define MOTOR_L_P               (20.37)
#define MOTOR_L_I               (72.19*0.005)
#define MOTOR_R_P               (23.43)
#define MOTOR_R_I               (83.36*0.005)


#define MOTOR_L_I_MAX           (0)                 //左速度环积分项最大值
#define MOTOR_R_I_MAX           (0)                 //右速度环积分项最大值
#define DIR_I_MAX               (0)                 //方向环积分项最大值
#define TARGET_SPEED_I_MAX      (0)                 //目标速度环积分项最大值
#define MOTOR_L_OUT_MAX             (9000)          //左速度环输出最大值
#define MOTOR_R_OUT_MAX             (9000)          //右速度环输出最大值
#define DIR_OUT_MAX                 (STEER_DUTY_LEFT-STEER_DUTY_MIDDLE)     //方向环输出最大值
#define TARGET_SPEED_OUT_MAX        (50)           //目标速度环输出最大值
#define MOTOR_L_OUT_MIN             (-9000)         //左速度环输出最小值
#define MOTOR_R_OUT_MIN             (-9000)         //右速度环输出最小值
#define DIR_OUT_MIN                 (STEER_DUTY_RIGHT-STEER_DUTY_MIDDLE)    //方向环输出最小值
#define TARGET_SPEED_OUT_MIN        (-50)           //目标速度环输出最小值


extern PID_Structure motor_l,motor_r,dir,target_speed_pid;


void PID_Init(PID_Structure* pid);                                                                                  //PID参数初始化
void PID_Config(PID_Structure* pid, float p, float i, float d, float i_max, float out_max, float out_min);      //PID参数配置
float PID_Position_Controller(PID_Structure* pid, float error);                                                     //位置式PID
float PID_Incremental_Controller(PID_Structure* pid, float error);                                                  //增量式PID

#endif
