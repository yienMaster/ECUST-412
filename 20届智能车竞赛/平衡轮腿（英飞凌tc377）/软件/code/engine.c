#include"engine.h"

/*************************************************************************
 *  函数名称：uint32 buu(uint32 c)
 *  功能说明：舵机初始化函数
 *  参数说明：
  * @param    c   ： 舵机占空比设置
 *  函数返回：无
 *  修改时间：2025年1月21日
 *  备    注：//设置占空比反向
 *************************************************************************/
uint32 buu(uint32 c)
{
    c = 1500 - c;
    return c;
}

/*************************************************************************
 *  函数名称：uint32 auu(uint32 c)
 *  功能说明：舵机初始化函数
 *  参数说明：
  * @param    c   ： 舵机占空比限幅设置
 *  函数返回：无
 *  修改时间：2025年1月21日
 *  备    注：//设置占空比限幅
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
 *  函数名称：void engine_init(int pwm);
 *  功能说明：舵机初始化函数
 *  参数说明：
  * @param    pwm   ： 舵机占空比初始化设置
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：//750———1250越大腿往下伸长
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
 *  函数名称：void engine_maintain(int pwm);
 *  功能说明：舵机保持函数
 *  参数说明：
  * @param    pwm   ： 舵机占空比输入
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：//750———1250越大腿往下伸长
 *************************************************************************/
void engine_maintain(int pwm1,int pwm2)//记得修改
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_1, buu(pwm1));
    pwm_set_duty(PWM_2, pwm2);
    pwm_set_duty(PWM_3, buu(pwm2));
    pwm_set_duty(PWM_4, pwm1);
}

/*************************************************************************
 *  函数名称：void engine_left_maintain(int pwm);
 *  功能说明：左舵机保持函数
 *  参数说明：
  * @param    pwm   ： 舵机占空比输入
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：//750———1250越大腿往下伸长
 *************************************************************************/
void engine_left_maintain(int pwm1,int pwm2)
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_1, buu(pwm1));
    pwm_set_duty(PWM_2, pwm2);
}

/*************************************************************************
 *  函数名称：void engine_right_maintain(int pwm);
 *  功能说明：右舵机保持函数
 *  参数说明：
  * @param    pwm   ： 舵机占空比输入
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：//750———1250越大腿往下伸长
 *************************************************************************/
void engine_right_maintain(int pwm1,int pwm2)
{
    pwm1 = auu(pwm1);
    pwm2 = auu(pwm2);
    pwm_set_duty(PWM_3, buu(pwm2));
    pwm_set_duty(PWM_4, pwm1);
}

/*************************************************************************
 *  函数名称：void engine_jump(void);
 *  功能说明：轮足跳跃函数
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：本历程尚未实现，建议改良为PID缓慢拟合
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
 *  函数名称：void engine_Stand_change(uint32 left, uint32 right, pid_param_t * pid);
 *  功能说明：轮足高度直立环选择函数
 *  函数返回：无
 *  修改时间：2025年1月18日
 *  备    注：本历程尚未实现，建议改良为PID缓慢拟合
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
