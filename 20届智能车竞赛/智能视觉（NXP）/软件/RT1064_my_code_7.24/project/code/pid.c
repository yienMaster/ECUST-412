#include "zf_common_headfile.h"
#include "pid.h"
#include "math.h"
/**
 * @brief 设置pid
 * @param void
 *  @see set_pid(Kp, Ki, Kd, motor1_pid);
 * @return void
 * 
 */
void set_pid(float Kp, float Ki, float Kd, pid_controller* pid)
{
		pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
		pid->Kp_high = 15.0f;
		pid->right = 0.3f;
		pid->d_prev = 0.0f;
    pid->e_prev = 0.0f;
    pid->e_prev_prev = 0.0f;
    pid->output_prev = 0.0f;
		pid->output_max = 2500;
		pid->output_min = -2500;
	  pid->adapt_pid_range = 60.0f;
}

/**
 * @brief 控制pid
 * @param float
 *  @see PID_control(target, current, motor1_pid);
 * @return pid控制值
 */
float PID_control(float target, float current, pid_controller *pid) {
    float e = target - current;
    
    
    // 计算增量（含滤波后的微分项）
    float d_term = pid->Kd * (e - 2*pid->e_prev + pid->e_prev_prev);
    d_term = pid->right * d_term + (1 - pid->right) * pid->d_prev; // 低通滤波（alpha=0.3）
    
    float delta_u = pid->Kp * (e - pid->e_prev) 
                  + pid->Ki * e 
                  + d_term;
    
    // 更新状态
    pid->d_prev = d_term;
    pid->e_prev_prev = pid->e_prev;
    pid->e_prev = e;
    
    // 输出限幅和抗饱和逻辑
    float output = pid->output_prev + delta_u;
    output = (output > pid->output_max) ? pid->output_max : 
             (output < pid->output_min) ? pid->output_min : output;
    
    // 仅在未饱和时更新output_prev
    if (output != pid->output_prev + delta_u) pid->output_prev = output; // 保持实际输出值
    else	pid->output_prev = output;
    
    
    return output;
}
//float PID_control(float target, float current, pid_controller *pid) {
//    float e = target - current;
//    
//    // 动态参数调整：大误差时增大Kp，小误差时正常Kp
//    float adaptive_Kp = (fabs(e) > pid->adapt_pid_range) ? pid->Kp_high : pid->Kp;
//    
//    // 计算增量（含滤波后的微分项）
//    float d_term = pid->Kd * (e - 2*pid->e_prev + pid->e_prev_prev);
//    d_term = pid->right * d_term + pid->right * pid->d_prev; // 低通滤波（alpha=0.3）
//    
//    float delta_u = adaptive_Kp * (e - pid->e_prev) 
//                  + pid->Ki * e 
//                  + d_term;
//    
//    // 更新状态
//    pid->d_prev = d_term;
//    pid->e_prev_prev = pid->e_prev;
//    pid->e_prev = e;
//    
//    // 输出限幅和抗饱和逻辑
//    float output = pid->output_prev + delta_u;
//    output = (output > pid->output_max) ? pid->output_max : 
//             (output < pid->output_min) ? pid->output_min : output;
//    
//    // 仅在未饱和时更新output_prev
//    if (output != pid->output_prev + delta_u) pid->output_prev = output; // 保持实际输出值
//    else	pid->output_prev = output;
//    
//    
//    return output;
//}
/*
float PID_control(float target, float current,pid_controller* pid)
{
	
		pid->error = target_vel - current_vel;
	
	  int integral = pid->error + pid->prev_integral;
		
		pid->derivative = pid->error - pid->prev_error;
		
		float output = pid->Kp * pid->error + pid->Ki * integral + pid->Kd * pid->derivative;
		pid->prev_error = pid->error;
		pid->prev_integral = integral;
		return output;
	

	int err = target_vel - current_vel;
	float filtVelocity = pid->right * err + (1 - pid->right) * pid->prev_error;
	float now_err = filtVelocity - pid->prev_error;
	pid->vel_sum += now_err;
	if(pid->vel_sum > 3000) pid->vel_sum = 3000;
	else if(pid->vel_sum < -3000) pid->vel_sum = -3000;
	pid->prev_error = now_err;
	pid->derivative = pid->error - pid->prev_error;
	float output = pid->Kp * now_err + pid->Ki * pid->vel_sum + pid->Kd * pid->derivative;

	return output;

}
*/