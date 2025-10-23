#include "zf_common_headfile.h"
#include "pid.h"

/*************************************************************************
 *  �������ƣ�float constrain_float(float amt, float low, float high)
 *  ����˵�����޷�����
 *  ����˵����
  * @param    amt   �� ����
  * @param    low   �� ���ֵ
  * @param    high  �� ���ֵ
 *  �������أ���
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));//С����Сֵ������Сֵ���������ֵ�������ֵ
}

// pid������ʼ������
void PidInit(pid_param_t * pid)//��ʼ��
{
    pid->kp        = 0;
    pid->ki        = 0;
    pid->kd        = 0;
    pid->imax      = 0;
    pid->out_p     = 0;
    pid->out_i     = 0;
    pid->out_d     = 0;
    pid->out       = 0;
    pid->integrator= 0;
    pid->last_error= 0;
    pid->last_derivative   = 0;
    pid->last_t    = 0;
}

//PID��������������
void PidChange(pid_param_t * pid, float p, float i,float d)//����PID
{
    pid->kp        = p;
    pid->ki        = i;
    pid->kd        = d;

}

/*************************************************************************
 *  �������ƣ�float PidLocCtrl(pid_param_t * pid, float error)
 *  ����˵����pidλ��ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* �ۻ���� */
    pid->integrator += error;

    /* ����޷� */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
/*************************************************************************
 *  �������ƣ�float PidIncCtrl(pid_param_t * pid, float error)
 *  ����˵����pid����ʽ���������
 *  ����˵����
  * @param    pid     pid����
  * @param    error   pid�������
 *  �������أ�PID������   ע���������Ѿ��������ϴν��
 *  �޸�ʱ�䣺2020��4��1��
 *  ��    ע��
 *************************************************************************/
float PidIncCtrl(pid_param_t * pid, float error)
{


    pid->out_p = pid->kp * (error - pid->last_error);
    pid->out_i = pid->ki * error;
    pid->out_d = pid->kd * ((error - 2.0f*(pid->last_error)) + pid->last_derivative);

    pid->last_derivative = error - pid->last_error;
    pid->last_error = error;

    pid->out += pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
/*�ŵ㣺
��λ��ʽPID��һ�ַǵ���ʽ�㷨����ֱ�ӿ���ִ�л�������ƽ��С������u(k)��ֵ��ִ�л�����ʵ��λ�ã���С����ǰ�Ƕȣ���һһ��Ӧ�ģ������ִ�л����������ֲ����Ķ����п��Ժܺ�Ӧ��

ȱ�㣺
��ÿ����������ȥ��״̬�йأ�����ʱҪ��e(k)�����ۼӣ����㹤������

����ʽPID��ȱ�㣺
�ŵ㣺
������ʱӰ��С����Ҫʱ�����߼��жϵķ���ȥ���������ݡ�
���ֶ�/�Զ��л�ʱ���С������ʵ�����Ŷ��л��������������ʱ�����ܱ���ԭֵ��
����ʽ�в���Ҫ�ۼӡ�����������u(k)��ȷ���������3�εĲ���ֵ�йء�


ȱ�㣺
�ٻ��ֽض�ЧӦ������̬��

�������Ӱ����еı��ض���������ʽ��̫�ã�

*/




