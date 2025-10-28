#include "PwmController.h"

int main() {
    // 步骤2: 创建PwmController对象，假设PWM芯片编号为0，PWM编号为1
    PwmController pwmController(0, 1);

    // 步骤3: 初始化PWM设备
    if (pwmController.initialize()) {
        std::cout << "PWM设备初始化成功！" << std::endl;

        // 步骤4: 进行PWM控制操作

        // 启用PWM设备
        if (pwmController.enable()) {
            std::cout << "PWM设备已启用！" << std::endl;

            // 设置PWM周期为1000000纳秒
            if (pwmController.setPeriod(1000000)) {
                std::cout << "PWM周期设置成功！" << std::endl;
            } else {
                std::cout << "PWM周期设置失败！" << std::endl;
            }

            // 设置PWM占空比为500000纳秒
            if (pwmController.setDutyCycle(500000)) {
                std::cout << "PWM占空比设置成功！" << std::endl;
            } else {
                std::cout << "PWM占空比设置失败！" << std::endl;
            }

            // 读取PWM周期
            int period = pwmController.readPeriod();
            if (period != -1) {
                std::cout << "当前PWM周期: " << period << " 纳秒" << std::endl;
            } else {
                std::cout << "读取PWM周期失败！" << std::endl;
            }

            // 读取PWM占空比
            int dutyCycle = pwmController.readDutyCycle();
            if (dutyCycle != -1) {
                std::cout << "当前PWM占空比: " << dutyCycle << " 纳秒" << std::endl;
            } else {
                std::cout << "读取PWM占空比失败！" << std::endl;
            }

            // 禁用PWM设备
            if (pwmController.disable()) {
                std::cout << "PWM设备已禁用！" << std::endl;
            } else {
                std::cout << "PWM设备禁用失败！" << std::endl;
            }
        } else {
            std::cout << "PWM设备启用失败！" << std::endl;
        }
    } else {
        std::cout << "PWM设备初始化失败！" << std::endl;
    }

    // 步骤5: 对象销毁时会自动调用析构函数
std::cout << "程序结束" << std::endl;
    return 0;
}