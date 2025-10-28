#include "PwmController.h"

// 构造函数实现
PwmController::PwmController(int pwmchip, int pwmnum)
    : pwmchip(pwmchip), pwmnum(pwmnum) {
    // 构建PWM设备的路径
    pwmPath = "/sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/pwm" + std::to_string(pwmnum) + "/";
}

// 析构函数实现，禁用PWM设备
PwmController::~PwmController() {
    disable();
}

// 读取PWM周期的实现
int PwmController::readPeriod() {
    return period;
}

// 读取PWM占空比的实现
int PwmController::readDutyCycle() {
    return duty_cycle;
}

// 初始化PWM设备的实现
bool PwmController::initialize() {
    std::string exportPath = "/sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/export";
    return writeToFile(exportPath, std::to_string(pwmnum));
}

// 启用PWM设备的实现
bool PwmController::enable() {
    return writeToFile(pwmPath + "enable", std::to_string(1));
}

// 禁用PWM设备的实现
bool PwmController::disable() {
    return writeToFile(pwmPath + "disable", std::to_string(0));
}

// 设置PWM周期的实现
bool PwmController::setPeriod(unsigned int period_ns) {
    period = period_ns;
    return writeToFile(pwmPath + "period", std::to_string(period_ns));
}

// 设置PWM占空比的实现
bool PwmController::setDutyCycle(unsigned int duty_cycle_ns) {
    duty_cycle = duty_cycle_ns;
    return writeToFile(pwmPath + "duty_cycle", std::to_string(duty_cycle_ns));
}

// 向文件写入值的辅助函数实现
bool PwmController::writeToFile(const std::string &path, const std::string &value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return false;
    }
    file << value;
    file.close();
    return true;
}