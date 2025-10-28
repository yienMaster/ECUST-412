#include "pwm.h"
#include <stddef.h>
#include <unistd.h>
#include <stdio.h>


int main() {
    PWM* pwm = pwm_init(0, 0); // 初始化 PWM，使用 pwmchip2 的通道gpio88 /TIM_CH2/
    if (pwm == NULL) {
        return 1;
    }
    if (!pwm_set_frequency(pwm, 50)) { // 设置频率为 1000Hz
        pwm_destroy(pwm);
        return 1;
    }
    if (!pwm_set_duty_cycle(pwm, 0.1)) { // 设置占空比为 10%
        pwm_destroy(pwm);
        return 1;
    }
    if (!pwm_enable(pwm)) { 
        pwm_destroy(pwm);
        return 1;
    }
    printf("已启动PWM\n");
    sleep(50); 
    pwm_disable(pwm); 
    pwm_destroy(pwm); 
    return 0;
}