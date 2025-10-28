

#include "servo.h"
#include <stdlib.h>
#include <math.h>

Servo* servo_init(int chip_num, int channel_num) {
    Servo* s = (Servo*)malloc(sizeof(Servo));
    if (!s) return NULL;

    s->pwm = pwm_init(chip_num, channel_num);
    if (!s->pwm) {
        free(s);
        return NULL;
    }

    // 设置 PWM 频率为 50Hz（周期 20ms = 20,000,000 ns）
    if (!pwm_set_frequency(s->pwm, SERVO_FREQUENCY)) {
        pwm_destroy(s->pwm);
        free(s);
        return NULL;
    }

    pwm_enable(s->pwm);
    return s;
}

bool servo_set_angle(Servo* s, float angle) {
    // 约束角度在 0~180 之间
    angle = fmaxf(0, fminf(180, angle));
    
    // 计算对应的脉冲宽度（单位：纳秒）
    float pulse_ns = SERVO_MIN_PULSE_NS + 
                    (angle / 180.0f) * 
                    (SERVO_MAX_PULSE_NS - SERVO_MIN_PULSE_NS);
    
    // 将脉冲宽度转换为占空比（占空比 = 脉冲宽度 / 周期）
    float duty_cycle = pulse_ns / (1000000000.0f / SERVO_FREQUENCY);
    
    return pwm_set_duty_cycle(s->pwm, duty_cycle);
}

void servo_destroy(Servo* s) {
    if (!s) return;
    pwm_disable(s->pwm);
    pwm_destroy(s->pwm);
    free(s);
}