#include "pid.h"
#include "mathfunc.h"

//PID参数结构体
PID_Structure motor_l,motor_r,dir;

float kp2,gkd;


void PID_Init(PID_Structure* pid)
{
   pid->p=0;
   pid->i=0;
   pid->d=0;
   pid->error0=0;
   pid->error00=0;
   pid->out=0;
   pid->integrator=0;
   pid->i_max=0;
   pid->out_max=0;
   pid->out_min=0;
}

void PID_Config(PID_Structure* pid, float p, float i, float d, float i_max, float out_max, float out_min)
{
    pid->p=p;
    pid->i=i;
    pid->d=d;
    pid->i_max=i_max;
    pid->out_max=out_max;
    pid->out_min=out_min;
}

float PID_Position_Controller(PID_Structure* pid,float error)
{
    float p_out,i_out,d_out;
    pid->integrator += error;   //更新积分项
    Constrain_int16(pid->integrator,-pid->i_max,pid->i_max);    //积分项限幅
    p_out = pid->p * error;
    i_out = pid->i * pid->integrator;
    d_out = pid->d * (error - pid->error0);
    pid->error00=pid->error0;
    pid->error0 = error;    //更新误差

    pid->out=p_out+i_out+d_out;    //pid输出
    pid->out=Constrain_float(pid->out,pid->out_max,pid->out_min);    //输出限幅
    return pid->out;
}

float PID_Incremental_Controller(PID_Structure* pid,float error)
{
    float p_out,i_out,d_out;
    p_out = pid->p * (error - pid->error0);
    i_out = pid->i * error;
    d_out = pid->d * (error - (pid->error0)/2 + pid->error00);
    pid->error00=pid->error0;
    pid->error0=error;      //更新误差
    pid->out+=p_out + i_out + d_out;
    pid->out=Constrain_float(pid->out,pid->out_max,pid->out_min);    //输出限幅
    return pid->out;
}

float Double_PD_Position_Controller(PID_Structure* pid,float error)
{
    float p_out,d_out;
    if(start_frontsight_count<3&&element_state==STRAIGHT)
    {
        p_out=pid->p * error;
    }
    else
    {
        p_out=pid->p * error+ dynamic_p_coef * error*error*error/1000;
    }
    Get_Gyro_Z();
    d_out=pid->d * (error - pid->error0) + gkd * gyro_z;
    pid->error0=error;
    pid->out =p_out+d_out;
    pid->out=Constrain_float(pid->out,pid->out_max,pid->out_min);    //输出限幅
    return pid->out;
}

void Para_Init(void)
{
    EEPROM_Write_Data(motor_l_p,32.770);
    EEPROM_Write_Data(motor_l_i,0.82); //1.02
    EEPROM_Write_Data(motor_r_p,32.519);
    EEPROM_Write_Data(motor_r_i,1.052); //1.002
    EEPROM_Write_Data(dir_kp2,0);
    para_update_flag=1;
    para_update_display_flag=1;
}


