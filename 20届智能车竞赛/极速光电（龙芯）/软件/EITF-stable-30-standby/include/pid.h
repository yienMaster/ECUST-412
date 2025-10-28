#ifndef PID_H
#define PID_H

#include <cmath>

// PID控制结构体
struct PID {
    double target;      // 目标值
    double kp;          // 比例系数
    double ki;          // 积分系数
    double integral;    // 积分项
    double last_error;  // 上次误差
    double kf;          // 前馈系数
    double filter_alpha;// 输出滤波系数
    double filtered_output; // 滤波状态
};

// 差速控制参数
extern float diff_speed;       // 动态差速值
extern const float DIFF_THRESHOLD; // 差速激活阈值
extern const float DIFF_KP;    // 差速比例系数

// PID功能函数
void pid_init(PID* pid, double kp, double ki, double kf, double fi, double target);
double pid_calculate(PID* pid, double feedback);

// 差速计算函数
void update_diff_speed(float current_gap, float current_pwm, float center_pwm);

#endif // PID_H