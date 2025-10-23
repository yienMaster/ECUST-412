#include "zf_common_headfile.h"
#include "pid.h"

/*************************************************************************
 *  函数名称：float constrain_float(float amt, float low, float high)
 *  功能说明：限幅函数
 *  参数说明：
  * @param    amt   ： 参数
  * @param    low   ： 最低值
  * @param    high  ： 最高值
 *  函数返回：无
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));//小于最小值就是最小值，大于最大值就是最大值
}

// pid参数初始化函数
void PidInit(pid_param_t * pid)//初始化
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

//PID三个参数的设置
void PidChange(pid_param_t * pid, float p, float i,float d)//设置PID
{
    pid->kp        = p;
    pid->ki        = i;
    pid->kd        = d;

}

/*************************************************************************
 *  函数名称：float PidLocCtrl(pid_param_t * pid, float error)
 *  功能说明：pid位置式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果
 *  修改时间：2020年4月1日
 *  备    注：
 *************************************************************************/
float PidLocCtrl(pid_param_t * pid, float error)
{
    /* 累积误差 */
    pid->integrator += error;

    /* 误差限幅 */
    constrain_float(pid->integrator, -pid->imax, pid->imax);


    pid->out_p = pid->kp * error;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (error - pid->last_error);

    pid->last_error = error;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;
}
/*************************************************************************
 *  函数名称：float PidIncCtrl(pid_param_t * pid, float error)
 *  功能说明：pid增量式控制器输出
 *  参数说明：
  * @param    pid     pid参数
  * @param    error   pid输入误差
 *  函数返回：PID输出结果   注意输出结果已经包涵了上次结果
 *  修改时间：2020年4月1日
 *  备    注：
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
/*优点：
①位置式PID是一种非递推式算法，可直接控制执行机构（如平衡小车），u(k)的值和执行机构的实际位置（如小车当前角度）是一一对应的，因此在执行机构不带积分部件的对象中可以很好应用

缺点：
①每次输出均与过去的状态有关，计算时要对e(k)进行累加，运算工作量大。

增量式PID优缺点：
优点：
①误动作时影响小，必要时可用逻辑判断的方法去掉出错数据。
②手动/自动切换时冲击小，便于实现无扰动切换。当计算机故障时，仍能保持原值。
③算式中不需要累加。控制增量Δu(k)的确定仅与最近3次的采样值有关。


缺点：
①积分截断效应大，有稳态误差；

②溢出的影响大。有的被控对象用增量式则不太好；

*/




