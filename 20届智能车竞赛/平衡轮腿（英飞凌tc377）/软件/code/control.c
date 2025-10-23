#include "control.h"
#include "FiveBarLinkageData.h"

// 全局变量
float target_velocity = 0.0;          // 目标速度
float now_velocity = 0.0;             //实际速度值
float target_motor_Stand = 2.2;       // 目标电机角度
float target_engine_high = 0.0;       // 目标发动机高度
float target_motor_angle = 0.0;       // 目标电机角度
float Encoder_Left, Encoder_Right; // 左右电机编码器值
extern float leg_error ;
int speed_up = 0;
float Turn_Pwm; // 转向PWM值
// PID参数
pid_param_t motor_speed;   // 速度PID参数
pid_param_t motor_Stand;   // 电机角度PID参数
pid_param_t motor_direction; // 方向PID参数
pid_param_t engine_high;   // 发动机高度PID参数
pid_param_t motor_gyro;    // 陀螺仪PID参数
pid_param_t air_roll_pid;    // 空中控制器参数
//static float left_angle, right_angle;
extern float x_current, y_current;
int i=0;
// 电机输出
signed short int Motor_Left, Motor_Right; // 左右电机PWM输出
float Velocity_Angle_left, Velocity_Angle_right; // 左右电机速度
// 其他变量
float last_error;                   // 上一次误差
extern Attitude_3D_Kalman filter;  // 卡尔曼滤波器
extern IMU_t IMU_data;             // IMU数据
extern float v_buchang;
float roll;                         // 倾斜角度
int engine_change = 600;            // 发动机变化量

// 腿部控制参数
#define MIN_LEG_LENGTH 0.04        // 最小腿部长度
#define MAX_LEG_LENGTH 0.1         // 最大腿部长度

// 模糊规则参数
typedef struct {
    float error_threshold_high;  // 高误差阈值
    float error_threshold_low;   // 低误差阈值
    float d_error_threshold_high; // 高误差变化率阈值
    float d_error_threshold_low;  // 低误差变化率阈值
    float kp_inc_high;           // 高KP增量
    float kp_inc_low;            // 低KP增量
    float ki_inc_high;           // 高KI增量
    float ki_inc_low;            // 低KI增量
    float kd_inc_high;           // 高KD增量
    float kd_inc_low;            // 低KD增量
} fuzzy_rules_t;

// PID限幅参数
typedef struct {
    float kp_max, kp_min;
    float ki_max, ki_min;
    float kd_max, kd_min;
} pid_limit_t;

// 默认模糊规则
fuzzy_rules_t speed_rules = {
    .error_threshold_high = 0.05,
    .error_threshold_low = -0.05,
    .d_error_threshold_high = 0.01,
    .d_error_threshold_low = -0.01,
    .kp_inc_high = 0.01,
    .kp_inc_low = 0.005,
    .ki_inc_high = 0.00001,
    .ki_inc_low = 0.000005,
    .kd_inc_high = 0.0005,
    .kd_inc_low = 0.00025
};
fuzzy_rules_t angle_rules = {
    .error_threshold_high = 8.0,  // 高误差阈值（度）
    .error_threshold_low = 3,  // 低误差阈值（度）
    .d_error_threshold_high =30.0, // 高误差变化率阈值（度/秒）
    .d_error_threshold_low = 5.0,  // 低误差变化率阈值（度/秒）
    .kp_inc_high =0.01,
    .kp_inc_low = 0.01,
    .ki_inc_high = 0,
    .ki_inc_low = 0,
    .kd_inc_high = 0.005,
    .kd_inc_low = 0.005
};
fuzzy_rules_t gyro_rules = {
    .error_threshold_high = 10,
    .error_threshold_low = -10,
    .d_error_threshold_high = 50,
    .d_error_threshold_low = 10,
    .kp_inc_high = 0.01,
    .kp_inc_low = 0.005,
    .ki_inc_high = 0,
    .ki_inc_low = 0,
    .kd_inc_high = 0.0005,
    .kd_inc_low = 0.00025
};

// 默认PID限幅参数
pid_limit_t angle_pid_limits = {15, 5, 0.2, 0, 0.5, 1.7};
pid_limit_t gyro_pid_limits = {2.0, 1, 1.3, 0.4, 0.5, 0.1};

// 模糊逻辑调整PID参数
void fuzzy_pid_adjust(pid_param_t *pid, float error, float d_error, fuzzy_rules_t *rules, pid_limit_t *limits) {
    // 模糊集合
    float e = error; // 误差
    float de = d_error; // 误差变化率

    // 模糊规则
    float delta_kp, delta_ki, delta_kd;

    // 根据误差和误差变化率调整PID参数
    if (fabs(e) > rules->error_threshold_high)
    {
        delta_kp = rules->kp_inc_high;
        delta_ki = 0;
        delta_kd = rules->kd_inc_high;
    }
    else if (fabs(e) < rules->error_threshold_low)
    {
        delta_kp = -rules->kp_inc_low;
        delta_ki = -rules->ki_inc_high;
        delta_kd = -rules->kd_inc_low;
    }
    else
       {
                  delta_kp = 0;
                  delta_ki =0;
                  delta_kd =0;
       }

    pid->kp += delta_kp;
    pid->ki += delta_ki;
    pid->kd += delta_kd;
    if (fabs(de) > rules->d_error_threshold_high)
    {
        delta_kp = +rules->kp_inc_high;
        delta_ki = 0;
        delta_kd = +rules->kd_inc_high;
    }
    else if (fabs(e) < rules->error_threshold_low)
       {
           delta_kp = rules->kp_inc_low;
           delta_ki = 0;
           delta_kd = +rules->kd_inc_low;
       }
    else
    {
               delta_kp = 0;
               delta_ki =0;
               delta_kd =0;
      }

    // 更新PID参数
    pid->kp += delta_kp;
    pid->ki += delta_ki;
    pid->kd += delta_kd;

    // 限制PID参数范围
    pid->kp = fmaxf(fminf(pid->kp, limits->kp_max), limits->kp_min);
    pid->ki = fmaxf(fminf(pid->ki, limits->ki_max), limits->ki_min);
    pid->kd = fmaxf(fminf(pid->kd, limits->kd_max), limits->kd_min);
}

// 初始化PID参数
void pid_init() {
    // 初始化速度PID
    PidInit(&motor_speed);
    PidChange(&motor_speed, 0.06, 0, 0.022);

    // 初始化电机角度PID
    PidInit(&motor_Stand);
    PidChange(&motor_Stand,6, 0, 0.4);

    // 初始化方向PID
    PidInit(&motor_direction);
    /*最低速 压弯3，单边桥350*/
    //PidChange(&motor_direction, 0.043, 0.00078,0.84);//600
    /*中速 压弯2，单边桥400*/
    PidChange(&motor_direction, 0.044, 0.00086,0.85);//650.1.83
    /*高速 压弯1，单边桥400*/
    //PidChange(&motor_direction, 0.048, 0.00090,0.86);//680.1.93
    /*高速 压弯1，单边桥400，时间空余可调*/
    //PidChange(&motor_direction, 0.050, 0.00092,0.86);//690
    //PidChange(&motor_direction, 0.056, 0.00092,0.88);//700

    // 初始化发动机高度PID
    PidInit(&engine_high);
    PidChange(&engine_high, 0.05, 0.05/200, 0);

    // 初始化陀螺仪PID
    PidInit(&motor_gyro);
    PidChange(&motor_gyro, 5, 0.5, 0.27);
    // 初始化空中控制器PID
    PidInit(&air_roll_pid);
    PidChange(&air_roll_pid, 45, 0, 2);//坚决不动
}
// 检测是否处于腾空状态
bool is_airborne() {

    const float threshold = 0.5;
    return fabs(IMU_data.accel[2] - 1) > threshold;
}
// 应用空中控制
void apply_air_control(float roll_control) {
    // 根据控制输出调整轮子转速
    float wheel_left = roll_control;
    float wheel_right = roll_control;

    // 限制轮子转速范围
    wheel_left = fmaxf(fminf(wheel_left, 1000.0), -1000.0);
    wheel_right = fmaxf(fminf(wheel_right, 1000.0), -1000.0);

    // 设置轮子转速
    small_driver_set_duty(wheel_left, wheel_right);
}

// 空中控制器主函数
void air_control() {
    static float last_roll_error = 0.0;
     // 获取当前姿态
    float current_roll = IMU_data.filter_result.roll;

    // 目标姿态（可以根据需要调整）
    float target_roll = 0.0;

    // 计算误差
    float roll_error = target_roll - current_roll;

    // 计算误差变化率
    float roll_d_error = roll_error - last_roll_error;

    // 保存当前误差
    last_roll_error = roll_error;
    //printf("data: %f,\r\n", current_roll);
    // 计算控制输出
    float roll_control = -air_roll_pid.kp * roll_error - air_roll_pid.ki * roll_error - air_roll_pid.kd * roll_d_error;

    // 限制输出范围
    roll_control = fmaxf(fminf(roll_control, 1000.0), -1000.0);

    // 应用控制输出到轮子
    apply_air_control(roll_control);
}


void adjust_pid_based_on_leg_height(float *current_leg_height) {
    // 计算腿部高度比例（0到1之间）
    float leg_ratio = (*current_leg_height - MIN_LEG_LENGTH) / (MAX_LEG_LENGTH - MIN_LEG_LENGTH);
    leg_ratio = fmaxf(fminf(leg_ratio, 1.0), 0.0);

    // 原始参数（腿部高度较低时）
    static bool initialized = false;
    static float original_angle_kp, original_angle_ki, original_angle_kd;
    static float original_gyro_kp, original_gyro_ki, original_gyro_kd,original_stand;

    if (!initialized) {
        original_angle_kp = 3;
        original_angle_ki = 0.001;
        original_angle_kd = 0.2;
        original_gyro_kp = 8;
        original_gyro_ki = 0.8;
        original_gyro_kd = 0.07;
        original_stand=4;
        initialized = true;
    }

    // 经验公式调整角度环PID参数
    float angle_kp = original_angle_kp * (1.0 - 0.3 * leg_ratio); // Kp减少
    float angle_ki = original_angle_ki; // Ki保持不变
    float angle_kd = original_angle_kd * (1.0 + 0.6 * leg_ratio); // Kd增加

    // 经验公式调整陀螺仪环PID参数
    float gyro_kp = original_gyro_kp * (1.0 - 0.2 * leg_ratio); // Kp减少
    float gyro_ki = original_gyro_ki * (1.0 - 0.4 * leg_ratio); // Ki减少
    float gyro_kd = original_gyro_kd * (1.0 + 0.4 * leg_ratio); // Kd增加
    // 默认PID限幅参数
    target_motor_Stand=original_stand*(1.0+0.4*leg_ratio);
    // 更新PID参数
    motor_Stand.kp = angle_kp;
    motor_Stand.ki = angle_ki;
    motor_Stand.kd = angle_kd;
    motor_gyro.kp = gyro_kp;
    motor_gyro.ki = gyro_ki;
    motor_gyro.kd = gyro_kd;
    pid_limit_t angle_pid_limits = {1.5*angle_kp, 0.5*angle_kp, 1.5*angle_ki, 0.5*angle_ki, 1.5*angle_kd, 0.5*angle_kd};
    pid_limit_t gyro_pid_limits = {1.5*gyro_kp, 0.5*gyro_kp, 1.5*gyro_kp, 0.5*gyro_kp, 1.5*gyro_kp, 0.5*gyro_kp};
}


// 初始化平衡控制
void Balance_init() {
    pid_init(); // 初始化PID参数
    Kalman_init(&filter, 1.0f, 0.05f, 0.05f, 0.05f, 0.05f, 0.05f); // 初始化卡尔曼滤波器
    int leg1, leg2;
    servo_control(x_current, y_current, &leg1, &leg2);
    engine_init(leg1, leg2); // 初始化发动机
}

// 平衡控制计算
float Balance(float Angle, float Gyro, float target) {
    float Angle_bias = target_motor_Stand + target - Angle; // 计算角度偏差
    float Gyro_bias = 0 - Gyro; // 计算陀螺仪偏差
    float balance = -motor_Stand.kp * Angle_bias - Gyro_bias * motor_Stand.kd; // 计算平衡值
    //
    //printf("data: %f,%f,%f\r\n", Angle_bias, Gyro_bias, balance);
   // fuzzy_pid_adjust(&motor_Stand, Angle_bias, Gyro_bias, &angle_rules, &angle_pid_limits);
    last_error = Angle_bias; // 更新误差
    if (balance > 4000) balance = 4000;
    if (balance < -4000) balance = -4000;
    return balance; // 返回平衡值
}



// 速度控制计算
float Velocity(int encoder_left, float target_velocity) {
    static float velocity; // 当前速度
    static float Encoder_bias; // 编码器偏差
    static float Encoder_Integral; // 积分项

    Encoder_bias = target_velocity - encoder_left; // 计算偏差
    Encoder_Integral += Encoder_bias; // 积分

    // 限制积分范围
    if (Encoder_Integral > 10000) Encoder_Integral = 10000;
    if (Encoder_Integral < -10000) Encoder_Integral = -10000;

    // 计算速度
    velocity = motor_speed.kp * Encoder_bias + motor_speed.ki * Encoder_Integral;
    // 限制速度范围
    if (velocity > 15) velocity = 15;
    if (velocity < -15) velocity = -15;

    // 动态调整PID参数
   // fuzzy_pid_adjust(&motor_speed, Encoder_bias, velocity - last_error, &speed_rules, &speed_pid_limits);

    return velocity; // 返回速度
}

// 陀螺仪控制计算
float GyroControl(float target_gyro, float current_gyro) {
    float gyro_error = target_gyro - current_gyro; // 计算陀螺仪误差
    static float gyro_Integral; // 积分项
    gyro_Integral += gyro_error; // 积分
    if (gyro_Integral > 5000) gyro_Integral = 5000;
        if (gyro_Integral < -5000) gyro_Integral = -5000;
    // 计算控制输出
    float gyro_control = +motor_gyro.kp * gyro_error + motor_gyro.ki * gyro_Integral + motor_gyro.kd * (gyro_error - last_error);
    last_error = gyro_error; // 更新误差

    // 动态调整PID参数
    //fuzzy_pid_adjust(&motor_gyro, gyro_error, gyro_error - last_error, &gyro_rules, &gyro_pid_limits);

    return -gyro_control; // 返回控制输出
}

// 限制PWM输出范围
int cuu(int c) {
    // 限制输出范围
    if (c > (MAX_DUTY * (PWM_DUTY_MAX / 100)))
        return (MAX_DUTY * (PWM_DUTY_MAX / 100));
    else if (c < -(MAX_DUTY * (PWM_DUTY_MAX / 100)))
        return -(MAX_DUTY * (PWM_DUTY_MAX / 100));
    else
        return c;
}

// 转向角度计算
float Turn_gyro(float target_angle, float gyro) {
    int y = 0;
    if (target_angle >= 0) {
        y = target_angle - 180;
        if (gyro <= y && gyro >= -180)
            gyro = 360 + gyro;
        gyro = gyro - target_angle;
    } else {
        y = target_angle + 180;
        if (gyro <= 180 && gyro >= y)
            gyro = gyro - 360;
        gyro = gyro - target_angle;
    }
    return gyro;
}

// 转向目标角度计算
float Turn_target(float target_angle) {
    if (target_angle >= 180)
        target_angle = target_angle - 360;
    else if (target_angle <= -180)
        target_angle = 360 + target_angle;
    return target_angle;
}

// 转向控制计算
float Turn(float gyro, float target_angle) {
    static float previous_error = 0.0; // 上一次误差
    float error = gyro - target_angle; // 当前误差
    float derivative = error - previous_error; // 微分项
    previous_error = error; // 更新误差

    // 计算控制输出
    float control_output = -motor_direction.kp * error - error * func_abs(error) * motor_direction.ki - motor_direction.kd * derivative;
    // 限制输出范围
    if (control_output > 2000) control_output = 2000;
    if (control_output < -2000) control_output = -2000;

    // 动态调整PID参数


    return control_output; // 返回控制输出
}

// 平衡控制主函数
void balance_control() {
    float Balance_Pwm_left, Balance_Pwm_right, Gyro_Pwm_left, Gyro_Pwm_right; // 平衡PWM值


    // 获取编码器值
    Encoder_Left = -motor_value.receive_left_speed_data;
    Encoder_Right = -motor_value.receive_right_speed_data;
    now_velocity  = (Encoder_Left - Encoder_Right)/2;

    // 更新倾斜角度
    roll = IMU_data.filter_result.roll ;
    i++;
    //跳跃更新
    if(jump_stop == 1)
    {
        // 初始化速度PID
        PidInit(&motor_speed);
        PidChange(&motor_speed, 0, 0, 0);

        // 初始化电机角度PID
        PidInit(&motor_Stand);
        PidChange(&motor_Stand,0, 0, 0);

    }
    else
    {
        // 初始化速度PID
        PidInit(&motor_speed);
        PidChange(&motor_speed, 0.06, 0, 0.022);

        // 初始化电机角度PID
        PidInit(&motor_Stand);
        PidChange(&motor_Stand,6, 0, 0.4);
    }
    // 计算左右电机速度
    if(i%5==0)
    {
        /********速度补偿单边桥启动***************************/
        if(leg_error>0)
        {
            Velocity_Angle_left = Velocity(Encoder_Left, target_velocity-v_buchang);
            Velocity_Angle_right = Velocity(-Encoder_Right, target_velocity+v_buchang);
        }
        else
        {
            Velocity_Angle_left = Velocity(Encoder_Left, target_velocity+v_buchang);
            Velocity_Angle_right = Velocity(-Encoder_Right, target_velocity-v_buchang);
        }
    }

//    printf("jiadata:%f,%f\r\n",Encoder_Left,target_velocity);
    // 平滑处理速度
//    left_angle = left_angle * 0.2 + Velocity_Angle_left * 0.8;
//    right_angle = right_angle * 0.2 + Velocity_Angle_right * 0.8;
    // 计算平衡PWM值
    if(i%2==0)
       {
        Balance_Pwm_left = Balance(roll, IMU_data.gyro[0], Velocity_Angle_left);
         Balance_Pwm_right = Balance(roll, IMU_data.gyro[0], Velocity_Angle_right);
       }
    // 计算陀螺仪控制PWM值

    Gyro_Pwm_left = GyroControl(-Balance_Pwm_left, IMU_data.gyro[0]);
    Gyro_Pwm_right = GyroControl(-Balance_Pwm_right, IMU_data.gyro[0]);


    // 计算最终PWM输出
    if(jump_position == 1 || target_velocity == 0)
    {
        Motor_Left = (signed short int)Gyro_Pwm_left;
        Motor_Right = (signed short int)Gyro_Pwm_right;
    }
    else
    {
        // 计算转向PWM值
        Turn_Pwm = Turn(border, 94);//中点拟合
        if(Turn_Pwm <= 0.2)
        {
            speed_up = 1;
        }
        else if(Turn_Pwm <= 2)
        {
            speed_up = 2;
        }
        else
        {
            speed_up = 0;
        }
        Motor_Left = (signed short int)Gyro_Pwm_left*(1+Turn_Pwm) - (signed short int)(imu963ra_gyro_z / 2);
        Motor_Right = (signed short int)Gyro_Pwm_right*(1-Turn_Pwm)  + (signed short int)(imu963ra_gyro_z / 2);
    }
    // 限制PWM输出范围
    Motor_Left = -(signed short int)cuu(Motor_Left);
    Motor_Right = (signed short int)cuu(Motor_Right);

    // 设置PWM输出
    small_driver_set_duty(Motor_Left, Motor_Right);
}

// 滤波处理
float filter_leg_control(float current_angle, float target_angle, float filter_factor) {
    return current_angle * filter_factor + target_angle * (1 - filter_factor);
}
/* 腿部控制器 */
float g_roll_int_gain = 0.000002f;  // 650以上变为1,600,3;650以下2
void leg_control(float *x, float *y)
{

        // 计算腿部位置
       // float x_cal = 0.000022 * (speed - target_velocity)+;
       // float x_cal = PidLocCtrl(&engine_high,speed - target_velocity);
        float x_cal=-0.035*tan((double)((Velocity_Angle_left+Velocity_Angle_right)/2/180*3.14));

//    /* ---------- 1. 前后速度环，计算公共基准高度 ---------- */
//    float x_cal = (Encoder_Left - Encoder_Right - 2.0f * target_velocity)
//                  * 0.013f / 2.0f / g_base_gain;
    if      (x_cal >  0.04f) x_cal =  0.04f;
    else if (x_cal < -0.04f) x_cal = -0.04f;

    *x = x_cal;          // 前后倾斜量（给舵机前后摆）
//
    /* ---------- 2. 左右平衡：单边抬高 ---------- */
    static float leg_error = 0.0f;
    static float angle_last = 0.0f;

    /* 2.1 低通滤波 & 误差计算 */
    float angle = 0.2f * angle_last + 0.8f * IMU_data.filter_result.pitch;
    angle_last = angle;

    float error = imu963ra_gyro_z;          // 目标 0.6° 以内算水平
    leg_error = g_roll_int_gain * error;

    /* 2.2 限幅：±25 mm 最大补偿 */
    if      (leg_error >  0.03f) leg_error =  0.03f;
    else if (leg_error < -0.03f) leg_error = -0.03f;

    /* ---------- 3. 计算前后基准高度 ---------- */
    float leg_target = *y;   // 你原本的前后基准高度

    /* ---------- 4. 叠加左右补偿：单边抬高 ---------- */
    float left_y  = leg_target;
    float right_y = leg_target;

    if (leg_error > 0)          // 向右倾 → 抬高右腿
        right_y = leg_target + leg_error;
    else if (leg_error < 0)     // 向左倾 → 抬高左腿
        left_y  =leg_target - leg_error;

    /* ---------- 5. 解算 + 输出 ---------- */
    int leg1, leg2, leg3, leg4;
    static int leg1_last, leg2_last, leg3_last, leg4_last;

    servo_control(*x, left_y,  &leg1, &leg2);   // 左腿
    servo_control(*x, right_y, &leg3, &leg4);   // 右腿

    /* 6. 一阶平滑 */
    leg1 = leg1 * 0.7f + leg1_last * 0.3f;
    leg2 = leg2 * 0.7f + leg2_last * 0.3f;
    leg3 = leg3 * 0.7f + leg3_last * 0.3f;
    leg4 = leg4 * 0.7f + leg4_last * 0.3f;

    engine_left_maintain (leg1, leg2);
    engine_right_maintain(leg3, leg4);

    leg1_last = leg1; leg2_last = leg2;
    leg3_last = leg3; leg4_last = leg4;
}
// //腿部控制
//void leg_control(float *x, float *y) {
////    float speed;
////    static float speed_last;
////    speed = (Encoder_Left - Encoder_Right) / 2;
////
////    // 速度滤波
////    if (fabs(speed_last - speed) < 10) {
////        speed = speed_last;
////    }
////    speed_last = speed;
//
//    // 计算腿部位置
//   // float x_cal = 0.000022 * (speed - target_velocity)+;
//   // float x_cal = PidLocCtrl(&engine_high,speed - target_velocity);
//    float x_cal=-0.04*tan((double)((Velocity_Angle_left+Velocity_Angle_right)/2/180*3.14));
//   // printf("jiadata:%f,%f\r\n",x_cal,((Velocity_Angle_left+Velocity_Angle_right)/2/180*3.14));
//    if(x_cal>0.05)
//    {
//        x_cal=0.05;
//    }  if(x_cal<-0.05)
//    {
//        x_cal=-0.05;
//    }
//    int leg1, leg2;
//    static int leg1_last, leg2_last;
//    if (fabs(x_cal - *x) > 0.001) {
//        *x = x_cal;
//        servo_control(*x, *y, &leg1, &leg2);
//        engine_left_maintain((int)leg1, (int)leg2);
//        engine_right_maintain((int)leg1, (int)leg2);
//        leg1 = leg1 * 0.8 + leg1_last * 0.2;
//        leg2 = leg2 * 0.8 + leg2_last * 0.2;
//        leg1_last = leg1;
//        leg2_last = leg2;
//    }
//}
// 速度补偿计算
float calculate_speed_compensation(float v, float h, float l) {
    // 计算单边桥上的轮子路程
    float S = sqrt(2 * h * h + (l / 2) * (l / 2));

    // 计算速度补偿
    float delta_v = (v * S - l) / l;

    return delta_v;
}
