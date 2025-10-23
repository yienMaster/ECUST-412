#include"engine.h"

/*************************************************************************
 *  �������ƣ�uint32 buu(uint32 c)
 *  ����˵���������ʼ������
 *  ����˵����
  * @param    c   �� ���ռ�ձ�����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��21��
 *  ��    ע��//����ռ�ձȷ���
 *************************************************************************/
uint32 buu(uint32 c)
{
    c = 1500 - c;
    return c;
}

/*************************************************************************
 *  �������ƣ�uint32 auu(uint32 c)
 *  ����˵���������ʼ������
 *  ����˵����
  * @param    c   �� ���ռ�ձ��޷�����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��21��
 *  ��    ע��//����ռ�ձ��޷�
 *************************************************************************/
uint32 auu(uint32 c)
 {
     if(c > 1200)
             return 1200;
     else if(c < 500)
             return 500;
     else
         return c;
 }

/*************************************************************************
 *  �������ƣ�void engine_init(int pwm);
 *  ����˵���������ʼ������
 *  ����˵����
  * @param    pwm   �� ���ռ�ձȳ�ʼ������
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע��//750������1250Խ���������쳤
 *************************************************************************/
void engine_init(int pwm1,int pwm2)
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_init(PWM_1, FREQ, buu(pwm1));
    pwm_init(PWM_2, FREQ, pwm2);
    pwm_init(PWM_3, FREQ, buu(pwm2));
    pwm_init(PWM_4, FREQ, pwm1);
}
/*************************************************************************
 *  �������ƣ�void engine_maintain(int pwm);
 *  ����˵����������ֺ���
 *  ����˵����
  * @param    pwm   �� ���ռ�ձ�����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע��//750������1250Խ���������쳤
 *************************************************************************/
void engine_maintain(int pwm1,int pwm2)//�ǵ��޸�
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_1, buu(pwm1));
    pwm_set_duty(PWM_2, pwm2);
    pwm_set_duty(PWM_3, buu(pwm2));
    pwm_set_duty(PWM_4, pwm1);
}

/*************************************************************************
 *  �������ƣ�void engine_left_maintain(int pwm);
 *  ����˵�����������ֺ���
 *  ����˵����
  * @param    pwm   �� ���ռ�ձ�����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע��//750������1250Խ���������쳤
 *************************************************************************/
void engine_left_maintain(int pwm1,int pwm2)
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_1, buu(pwm1));
    pwm_set_duty(PWM_2, pwm2);
}

/*************************************************************************
 *  �������ƣ�void engine_right_maintain(int pwm);
 *  ����˵�����Ҷ�����ֺ���
 *  ����˵����
  * @param    pwm   �� ���ռ�ձ�����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע��//750������1250Խ���������쳤
 *************************************************************************/
void engine_right_maintain(int pwm1,int pwm2)
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_3, buu(pwm2));
    pwm_set_duty(PWM_4, pwm1);
}

/*************************************************************************
 *  �������ƣ�void engine_jump(void);
 *  ����˵����������Ծ����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע����������δʵ�֣��������ΪPID�������
 *************************************************************************/
void engine_jump(void)
{
//    engine_maintain(750);
//    system_delay_ms(100);
//    engine_maintain(1250);
//    system_delay_ms(100);
//    engine_maintain(750);
}

/*************************************************************************
 *  �������ƣ�void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid);
 *  ����˵��������߶�ֱ����ѡ����
 *  �������أ���
 *  �޸�ʱ�䣺2025��1��18��
 *  ��    ע����������δʵ�֣��������ΪPID�������
 *************************************************************************/
void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid1, pid_param_t * pid2)
{
    if(left >right)
    {
        auu(left);
        if(left == 750)
            PidChange(pid1, 17000, 0, 1570);
        else if(left>750 && left<850)
        {
            PidChange(pid1, 18500, 0, 1570);
            target_motor_Stand = 0;
        }
        else if(left>=850 && left<950)
        {
            PidChange(pid1, 20500, 0, 1940);
            PidChange(pid2,5,5/200, 0);
            target_motor_Stand = 0;
        }
        else
            PidChange(pid1, 24000, 0, 1570);
    }
    else
    {
        auu(right);
        if(left == 750)
           PidChange(pid1, 17000, 0, 1570);
        else if(right>750 && right<850)
        {
            PidChange(pid1, 18500, 0, 1570);
            target_motor_Stand = 0;
        }
        else if(right>=850 && right<950)
        {
            PidChange(pid1, 20500, 0, 1940);
            PidChange(pid2,5,5/200, 0);
            target_motor_Stand = 0;
        }
        else
        {
            PidChange(pid1, 22500, 0, 2180);
            PidChange(pid2,5.3,5.3/200, 0);
            target_motor_Stand = 0;
        }
    }
}
