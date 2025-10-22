#ifndef __PID_H__
#define __PID_H__

#include "zf_common_headfile.h"



//PID�ṹ��
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


#define MOTOR_L_I_MAX           (0)                 //���ٶȻ����������ֵ
#define MOTOR_R_I_MAX           (0)                 //���ٶȻ����������ֵ
#define DIR_I_MAX               (0)                 //���򻷻��������ֵ
#define MOTOR_L_OUT_MAX             (8000)          //���ٶȻ�������ֵ
#define MOTOR_R_OUT_MAX             (8000)          //���ٶȻ�������ֵ
#define DIR_OUT_MAX                 (200)           //����������ֵ
#define MOTOR_L_OUT_MIN             (-8000)         //���ٶȻ������Сֵ
#define MOTOR_R_OUT_MIN             (-8000)         //���ٶȻ������Сֵ
#define DIR_OUT_MIN                 (-200)          //���������Сֵ


extern PID_Structure motor_l,motor_r,dir;
extern float kp2,gkd;


void PID_Init(PID_Structure* pid);                                                                                  //PID������ʼ��
void PID_Config(PID_Structure* pid, float p, float i, float d, float i_max, float out_max, float out_min);      //PID��������
float PID_Position_Controller(PID_Structure* pid, float error);                                                     //λ��ʽPID
float PID_Incremental_Controller(PID_Structure* pid, float error);                                                  //����ʽPID
float PID_Position_Controller(PID_Structure* pid,float error);
float Double_PD_Position_Controller(PID_Structure* pid,float error);
void Para_Init(void);

#endif
