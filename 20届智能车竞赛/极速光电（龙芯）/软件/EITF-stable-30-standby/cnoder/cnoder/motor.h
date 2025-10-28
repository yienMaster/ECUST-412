#ifdef __cplusplus
extern "C" {
#endif
#ifndef MOTOR_H
#define MOTOR_H

#include "pwm.h"

typedef struct {
    PWM* pwm_speed;     // 速度控制PWM通道
    PWM* pwm_direction; // 方向控制PWM通道（可选）
    int current_speed;  // 当前速度值（0-100）
} Motor;

Motor* motor_init(int speed_chip, int speed_channel);
bool motor_set_speed(Motor* motor, int speed);
void motor_destroy(Motor* motor);
bool motor_set_frequency(Motor* motor, int frequency);
#endif
#ifdef __cplusplus
};
#endif