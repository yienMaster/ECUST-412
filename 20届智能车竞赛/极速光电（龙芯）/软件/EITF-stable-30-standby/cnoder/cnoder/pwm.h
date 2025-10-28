#ifdef __cplusplus
extern "C" {
#endif
#ifndef PWM_H
#define PWM_H

#include <stdbool.h>

// PWM 结构体，用于存储 PWM 相关信息
typedef struct {
    int chip_num;  // PWM 控制器编号
    int channel_num; // PWM 通道编号
} PWM;


/*
@brief  初始化 PWM 结构体
@param  chip_num  PWM 控制器编号
@param  channel_num  PWM 通道编号
@return  PWM*  成功返回指向 PWM 结构体的指针，失败返回 NULL
*/
PWM* pwm_init(int chip_num, int channel_num);


/*
@brief  设置 PWM 的频率
@param  pwm  指向 PWM 结构体的指针
@param  frequency  要设置的频率值（单位：Hz）
@return  bool  成功返回 true，失败返回 false
*/
bool pwm_set_frequency(PWM* pwm, unsigned int frequency);


/*
@brief  设置 PWM 的占空比
@param  pwm  指向 PWM 结构体的指针
@param  duty_cycle  要设置的占空比（范围：0.0 到 1.0）
@return  bool  成功返回 true，失败返回 false
*/
bool pwm_set_duty_cycle(PWM* pwm, float duty_cycle);


/*
@brief  启用 PWM
@param  pwm  指向 PWM 结构体的指针
@return  bool  成功返回 true，失败返回 false
*/
bool pwm_enable(PWM* pwm);


/*
@brief  禁用 PWM
@param  pwm  指向 PWM 结构体的指针
@return  bool  成功返回 true，失败返回 false
*/
bool pwm_disable(PWM* pwm);


/*
@brief  销毁 PWM 结构体，释放资源
@param  pwm  指向 PWM 结构体的指针
*/
void pwm_destroy(PWM* pwm);


#endif
#ifdef __cplusplus
};
#endif