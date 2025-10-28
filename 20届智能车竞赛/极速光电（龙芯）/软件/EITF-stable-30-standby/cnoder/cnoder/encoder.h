#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#include "gpio.h"
#include "register.h"

// 编码器线数
//#define ENCODER_PPR   1024

// PWM控制器的基地址和偏移量
#define PWM_BASE_ADDR 0x1611B000//从chip4到7为0，1，2，3
#define PWM_OFFSET 0x10
#define LOW_BUFFER_OFFSET 0x4
#define FULL_BUFFER_OFFSET 0x8
#define CONTROL_REG_OFFSET 0xC

// 控制寄存器的位定义
#define CNTR_ENABLE_BIT (1 << 0)       // 计数器使能//1
#define PULSE_OUT_ENABLE_BIT (1 << 3)  // 脉冲输出使能（低有效）
#define SINGLE_PULSE_BIT (1 << 4)      // 单脉冲控制位
#define INT_ENABLE_BIT (1 << 5)        // 中断使能//1
#define INT_STATUS_BIT (1 << 6)        // 中断状态
#define COUNTER_RESET_BIT (1 << 7)     // 计数器重置
#define MEASURE_PULSE_BIT (1 << 8)     // 测量脉冲使能//1
#define INVERT_OUTPUT_BIT (1 << 9)     // 输出翻转使能
#define DEAD_ZONE_ENABLE_BIT (1 << 10) // 防死区使能

// ENCODER类，用于控制编码器
class ENCODER {
public:
    // 构造函数，接受PWM通道编号和GPIO编号作为参数
    ENCODER(int pwmNum, int gpioNum);
    // 析构函数，释放映射的内存
    ~ENCODER(void);

    // 更新编码器的脉冲计数并返回RPS
    double pulse_counter_update(void);

private:
    uint32_t base_addr;  // PWM控制器的基地址
    GPIO directionGPIO;  // 方向控制的GPIO对象
    void *low_buffer;    // 低缓冲区的映射地址
    void *full_buffer;   // 完整缓冲区的映射地址
    void *control_buffer; // 控制寄存器的映射地址

    // 初始化PWM控制器为计数模式
    void PWM_Init(void);
    // 清空计数器
    void reset_counter(void);
};

#endif   