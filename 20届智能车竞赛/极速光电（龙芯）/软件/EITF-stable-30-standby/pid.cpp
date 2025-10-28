#include "pid.h"

// 差速控制参数
float diff_speed = 0.0;
const float DIFF_THRESHOLD = 50.0;
const float DIFF_KP = 0.4;

// PID初始化
void pid_init(PID* pid, double kp, double ki, double kf, double fi, double target) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kf = kf;
    pid->filter_alpha = fi;
    pid->filtered_output = 0;
    pid->target = target;
    pid->integral = 0;
    pid->last_error = 0;
}

// PID计算
double pid_calculate(PID* pid, double feedback) {
    const double integral_max = 25.0;
    const double output_max = 100;
    const double output_min = 0;

    double error = pid->target - feedback;
    double feedforward = pid->kf * pid->target;
    
    // 积分抗饱和
    pid->integral += error;
    pid->integral = fmin(fmax(pid->integral, -integral_max), integral_max);
    
    // 动态积分衰减
    if(fabs(error) < 0.15) {
        double alpha = (fabs(error) < 0.03) ? 0.96 : 
                     0.8 + (0.15 - fabs(error))/0.15*0.15;
        pid->integral *= alpha;
    }
    
    // 综合计算
    double output = feedforward + 
                   pid->kp * error + 
                   pid->ki * pid->integral;
    
    // 输出滤波
    pid->filtered_output = pid->filter_alpha * pid->filtered_output + 
                          (1 - pid->filter_alpha) * output;
    
    return fmax(fmin(pid->filtered_output, output_max), output_min);
}

// 差速更新（在图像处理循环中调用）
void update_diff_speed(float current_gap, float current_pwm, float center_pwm = 0.45f) {
    if(fabs(current_gap) > DIFF_THRESHOLD) {
        float steer_ratio = (current_pwm - center_pwm) / 0.04f;
        diff_speed = DIFF_KP * steer_ratio * 15.0; // 基础速度硬编码为15
    } else {
        diff_speed = 0.0;
    }
}