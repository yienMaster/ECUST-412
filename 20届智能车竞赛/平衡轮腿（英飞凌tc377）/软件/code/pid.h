#ifndef CODE_PID_H_
#define CODE_PID_H_

typedef struct
{
    float                kp;         //P
    float                ki;         //I
    float                kd;         //D
    float                imax;       //�����޷�

    float                out_p;  //KP���
    float                out_i;  //KI���
    float                out_d;  //KD���
    float                out;    //pid���

    float                integrator; //< ����ֵ
    float                last_error; //< �ϴ����
    float                last_derivative;//< �ϴ���������ϴ����֮��
    unsigned long        last_t;     //< �ϴ�ʱ��
}pid_param_t;


void PidInit(pid_param_t * pid);

void PidChange(pid_param_t * pid, float p, float i,float d);

float constrain_float(float amt, float low, float high);

float PidLocCtrl(pid_param_t * pid, float error);

float PidIncCtrl(pid_param_t * pid, float error);





#endif /* CODE_PID_H_ */
