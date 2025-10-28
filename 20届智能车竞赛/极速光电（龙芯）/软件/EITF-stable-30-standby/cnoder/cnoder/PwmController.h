#ifndef PWM_CONTROLLER_H
#define PWM_CONTROLLER_H

#include <string>
#include <fstream>
#include <iostream>

// PwmController类，用于控制PWM设备
class PwmController {
public:
    // 构造函数，接受PWM芯片编号和PWM编号作为参数
    PwmController(int pwmchip, int pwmnum);
    // 析构函数，禁用PWM设备
    ~PwmController();

    // 启用PWM设备
    bool enable();
    // 禁用PWM设备
    bool disable();
    // 设置PWM周期（以纳秒为单位）
    bool setPeriod(unsigned int period_ns);
    // 设置PWM占空比（以纳秒为单位）
    bool setDutyCycle(unsigned int duty_cycle_ns);
    // 初始化PWM设备
    bool initialize();
    // 读取PWM周期
    int readPeriod();
    // 读取PWM占空比
    int readDutyCycle();

private:
    std::string pwmPath;  // PWM设备的路径
    int pwmchip;          // PWM芯片编号
    int pwmnum;           // PWM编号
    int period;           // PWM周期
    int duty_cycle;       // PWM占空比

    // 辅助函数，用于向文件写入值
    bool writeToFile(const std::string &path, const std::string &value);
};

#endif