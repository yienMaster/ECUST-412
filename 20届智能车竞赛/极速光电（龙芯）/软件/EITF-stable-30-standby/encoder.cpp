#include "encoder.h"

// 构造函数实现
ENCODER::ENCODER(int pwmNum, int gpioNum) : base_addr(PWM_BASE_ADDR + pwmNum * PWM_OFFSET), directionGPIO(gpioNum) {
    // 设置方向控制GPIO为输入模式
    directionGPIO.setDirection("in");

    // 映射控制寄存器、低缓冲区和完整缓冲区
    control_buffer = map_register(base_addr + CONTROL_REG_OFFSET, PAGE_SIZE);
    low_buffer = map_register(base_addr + LOW_BUFFER_OFFSET, PAGE_SIZE);
    full_buffer = map_register(base_addr + FULL_BUFFER_OFFSET, PAGE_SIZE);
    if (full_buffer == MAP_FAILED) {
        perror("Failed to map full_buffer");
        exit(EXIT_FAILURE);
    }

    printf("Registers mapped successfully\n");

    // 初始化PWM控制器
    PWM_Init();
}

// 析构函数实现，释放映射的内存
ENCODER::~ENCODER(void) {
    munmap(control_buffer, PAGE_SIZE);
    munmap(low_buffer, PAGE_SIZE);
    munmap(full_buffer, PAGE_SIZE);
}

// 初始化PWM控制器为计数模式的实现
void ENCODER::PWM_Init(void) {
    uint32_t control_reg = 0;

    // 设置控制寄存器的相关位
    control_reg |= CNTR_ENABLE_BIT;
    control_reg |= MEASURE_PULSE_BIT;
    control_reg |= INT_ENABLE_BIT;

    // 写入控制寄存器
    REG_WRITE(control_buffer, control_reg);

    printf("PWM initialized with control register: 0x%08X\n", control_reg);
}

// 清空计数器的实现
void ENCODER::reset_counter(void) {
    uint32_t control_reg = REG_READ(control_buffer);
    control_reg |= COUNTER_RESET_BIT;
    REG_WRITE(control_buffer, control_reg);
}

// 更新编码器的脉冲计数并返回RPS的实现
double ENCODER::pulse_counter_update(void) {
    uint32_t pulse_count = REG_READ(full_buffer);
    int direction = directionGPIO.readValue();
    //printf("Pulse count: %u, Direction: %d\n", pulse_count, direction);
    double value = 100000000.0 / pulse_count / 1024 * (direction * 2 - 1);
    return value;
}
// 初始化电机方向控制GPIO
bool motor_init_direction(Motor* motor) {
    GPIO directionGPIO(motor->direction_channel);  // 使用GPIO编号初始化
    if (!directionGPIO.setDirection("out")) {      // 设置GPIO为输出模式
        return false;
    }
    return true;
}

// 设置电机方向
bool motor_set_direction(Motor* motor, int control_direction) {
    GPIO directionGPIO(motor->direction_channel);  // 使用GPIO编号初始化
    bool direction = control_direction;            // 直接使用传入的方向参数
    if (!directionGPIO.setValue(direction)) {
        return false;
    }
    return true;
}