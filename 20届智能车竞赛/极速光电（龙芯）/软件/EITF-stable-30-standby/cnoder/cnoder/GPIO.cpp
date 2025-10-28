#include "GPIO.h"

// 构造函数实现
GPIO::GPIO(int gpioNum_) : gpioNum(gpioNum_), fd(-1) {
    // 构建GPIO设备文件的路径
    gpioPath = "/sys/class/gpio/gpio" + std::to_string(gpioNum);

    // 导出GPIO，使其可以被用户空间访问
    if (!writeToFile("/sys/class/gpio/export", std::to_string(gpioNum))) {
        throw std::runtime_error("Failed to export GPIO " + std::to_string(gpioNum));
    }

    // 打开GPIO的value文件，以读写方式打开
    fd = open((gpioPath + "/value").c_str(), O_RDWR);
    if (fd == -1) {
        throw std::runtime_error("Failed to open GPIO value file: " + std::string(strerror(errno)));
    }
}

// 析构函数实现，关闭文件描述符
GPIO::~GPIO(void) {
    close(fd);
}

// 设置GPIO方向的实现
bool GPIO::setDirection(const std::string &direction) {
    return writeToFile(gpioPath + "/direction", direction);
}

// 设置GPIO触发边沿的实现
bool GPIO::setEdge(const std::string &edge) {
    return writeToFile(gpioPath + "/edge", edge);
}

// 设置GPIO输出值的实现
bool GPIO::setValue(bool value) {
    const char *val_str = value ? "1" : "0";
    // 使用文件描述符写入GPIO值
    if (write(fd, val_str, 1) != 1) {
        std::cerr << "Failed to write GPIO value: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

// 读取GPIO输入值的实现
bool GPIO::readValue(void) {
    char value;
    // 重置文件偏移量
    lseek(fd, 0, SEEK_SET);
    // 读取GPIO值
    if (read(fd, &value, 1) != 1) {
        std::cerr << "Failed to read GPIO value: " << strerror(errno) << std::endl;
        return false;
    }
    return value == '1';
}

// 获取GPIO文件描述符的实现
int GPIO::getFileDescriptor(void) const {
    return fd;
}

// 向文件写入值的辅助函数实现
bool GPIO::writeToFile(const std::string &path, const std::string &value) {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "Failed to open file " << path << ": " << strerror(errno) << std::endl;
        return false;
    }
    file << value;
    return file.good();
}