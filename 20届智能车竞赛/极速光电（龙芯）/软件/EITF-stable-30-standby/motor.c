#include "motor.h"
#include <stdlib.h>
#include "pwm.h"

// 电机工作频率 (17KHz)
#define MOTOR_FREQUENCY 17000
// 电机最小占空比 (1%)
#define MOTOR_MIN_DUTY 0.01

Motor* motor_init(int speed_chip, int speed_channel) {
    Motor* motor = (Motor*)malloc(sizeof(Motor));
    if (!motor) return NULL;

    motor->pwm_speed = pwm_init(speed_chip, speed_channel);
    if (!motor->pwm_speed) {
        free(motor);
        return NULL;
    }

    // 设置电机工作频率
    if (!pwm_set_frequency(motor->pwm_speed, MOTOR_FREQUENCY)) {
        pwm_destroy(motor->pwm_speed);
        free(motor);
        return NULL;
    }

    motor->current_speed = 0;
    pwm_enable(motor->pwm_speed);
    return motor;
}
bool motor_set_frequency(Motor* motor, int frequency) {
    if (frequency <= 0) return false;
    if (!pwm_set_frequency(motor->pwm_speed, frequency)) return false;
    return true; 
}
bool motor_set_speed(Motor* motor, int speed) {
    if (speed < 0) speed = 0;
    if (speed > 75) speed = 75;  //0719 提升速度上限到75

    // 计算占空比，确保不低于 1%
    float duty_cycle = (speed / 100.0f);
    if (duty_cycle < MOTOR_MIN_DUTY) {
        duty_cycle = MOTOR_MIN_DUTY;
    }

    if (!pwm_set_duty_cycle(motor->pwm_speed, duty_cycle)) {
        return false;
    }

    motor->current_speed = speed;
    return true;
}

void motor_destroy(Motor* motor) {
    if (!motor) return;
    pwm_disable(motor->pwm_speed);
    pwm_destroy(motor->pwm_speed);
    free(motor);
}