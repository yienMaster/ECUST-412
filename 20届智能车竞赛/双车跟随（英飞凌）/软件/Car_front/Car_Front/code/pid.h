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


#define MOTOR_L_I_MAX           (0)                 //左速度环积分项最大值
#define MOTOR_R_I_MAX           (0)                 //右速度环积分项最大值
#define DIR_I_MAX               (0)                 //方向环积分项最大值
#define MOTOR_L_OUT_MAX             (8000)          //左速度环输出最大值
#define MOTOR_R_OUT_MAX             (8000)          //右速度环输出最大值
#define DIR_OUT_MAX                 (200)           //方向环输出最大值
#define MOTOR_L_OUT_MIN             (-8000)         //左速度环输出最小值
#define MOTOR_R_OUT_MIN             (-8000)         //右速度环输出最小值
#define DIR_OUT_MIN                 (-200)          //方向环输出最小值


extern PID_Structure motor_l,motor_r,dir;
extern float kp2,gkd;


void PID_Init(PID_Structure* pid);                                                                                  //PID参数初始化
void PID_Config(PID_Structure* pid, float p, float i, float d, float i_max, float out_max, float out_min);      //PID参数配置
float PID_Position_Controller(PID_Structure* pid, float error);                                                     //位置式PID
float PID_Incremental_Controller(PID_Structure* pid, float error);                                                  //增量式PID
float PID_Position_Controller(PID_Structure* pid,float error);
float Double_PD_Position_Controller(PID_Structure* pid,float error);
void Para_Init(void);

#endif
