#ifdef __cplusplus
extern "C" {
#endif
#ifndef SERVO_H
#define SERVO_H

#include "pwm.h"

// 定义舵机参数
#define SERVO_MIN_PULSE_NS  1000000  // 1ms 对应 0°
#define SERVO_MAX_PULSE_NS  2000000  // 2ms 对应 180°
#define SERVO_FREQUENCY     50       // 50Hz 周期为 20ms

typedef struct {
    PWM* pwm;
    float current_angle;
} Servo;

Servo* servo_init(int chip_num, int channel_num);
bool servo_set_angle(Servo* servo, float angle);
void servo_destroy(Servo* servo);

#endif
#ifdef __cplusplus
};
#endif
