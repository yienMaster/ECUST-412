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


#define MOTOR_L_P               (20.37)
#define MOTOR_L_I               (72.19*0.005)
#define MOTOR_R_P               (23.43)
#define MOTOR_R_I               (83.36*0.005)


#define MOTOR_L_I_MAX           (0)                 //���ٶȻ����������ֵ
#define MOTOR_R_I_MAX           (0)                 //���ٶȻ����������ֵ
#define DIR_I_MAX               (0)                 //���򻷻��������ֵ
#define TARGET_SPEED_I_MAX      (0)                 //Ŀ���ٶȻ����������ֵ
#define MOTOR_L_OUT_MAX             (9000)          //���ٶȻ�������ֵ
#define MOTOR_R_OUT_MAX             (9000)          //���ٶȻ�������ֵ
#define DIR_OUT_MAX                 (STEER_DUTY_LEFT-STEER_DUTY_MIDDLE)     //����������ֵ
#define TARGET_SPEED_OUT_MAX        (50)           //Ŀ���ٶȻ�������ֵ
#define MOTOR_L_OUT_MIN             (-9000)         //���ٶȻ������Сֵ
#define MOTOR_R_OUT_MIN             (-9000)         //���ٶȻ������Сֵ
#define DIR_OUT_MIN                 (STEER_DUTY_RIGHT-STEER_DUTY_MIDDLE)    //���������Сֵ
#define TARGET_SPEED_OUT_MIN        (-50)           //Ŀ���ٶȻ������Сֵ


extern PID_Structure motor_l,motor_r,dir,target_speed_pid;


void PID_Init(PID_Structure* pid);                                                                                  //PID������ʼ��
void PID_Config(PID_Structure* pid, float p, float i, float d, float i_max, float out_max, float out_min);      //PID��������
float PID_Position_Controller(PID_Structure* pid, float error);                                                     //λ��ʽPID
float PID_Incremental_Controller(PID_Structure* pid, float error);                                                  //����ʽPID

#endif
