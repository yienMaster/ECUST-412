#ifndef GPIO_H
#define GPIO_H

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

// GPIO类，用于控制GPIO引脚
class GPIO {
public:
    // 构造函数，接受GPIO编号作为参数
    GPIO(int gpioNum_);
    // 析构函数，关闭文件描述符
    ~GPIO(void);

    // 获取GPIO文件描述符
    int getFileDescriptor(void) const;
    // 设置GPIO方向，"out"为输出，"in"为输入
    bool setDirection(const std::string &direction);
    // 设置GPIO输出值
    bool setValue(bool value);
    // 读取GPIO输入值
    bool readValue(void);
    // 设置GPIO的触发边沿
    bool setEdge(const std::string &edge);

private:
    int gpioNum;  // GPIO编号
    int fd;       // 文件描述符，用于操作GPIO的value文件
    std::string gpioPath;  // GPIO设备文件的路径

    // 辅助函数，用于向文件写入值
    bool writeToFile(const std::string &path, const std::string &value);
};

#endif // GPIO_H