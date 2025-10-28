#include "pwm.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>


#define PWM_PATH_FORMAT "/sys/class/pwm/pwmchip%d/pwm%d"

//定义路径
#define EXPORT_PATH_FORMAT "/sys/class/pwm/pwmchip%d/export"
#define UNEXPORT_PATH_FORMAT "/sys/class/pwm/pwmchip%d/unexport"
#define PERIOD_PATH_FORMAT "%s/period"
#define DUTY_CYCLE_PATH_FORMAT "%s/duty_cycle"
#define ENABLE_PATH_FORMAT "%s/enable"


// 将整数转换为字符串
static void int_to_str(char* str, int num) {
    snprintf(str, 10, "%d", num);
}


// 将浮点数转换为字符串
static void float_to_str(char* str, float num) {
    snprintf(str, 30, "%.0f", num * 1000000);
}


// 将无符号整数转换为字符串
static void uint_to_str(char* str, unsigned int num) {
    snprintf(str, 20, "%u", num);
}


// 导出 PWM 通道
static bool pwm_export(PWM* pwm) {
    char export_path[100];
    int fd;
    char num_str[10];
    int_to_str(num_str, pwm->channel_num);
    snprintf(export_path, sizeof(export_path), EXPORT_PATH_FORMAT, pwm->chip_num);
    printf("pwm_export: export_path = %s\n", export_path); 
    fd = open(export_path, O_WRONLY); // 移除 O_TRUNC 标志
    if (fd == -1) {
        perror("pwm_export: open export_path failed");
        return false;
    }
    if (write(fd, num_str, strlen(num_str)) == -1) {
        perror("pwm_export: write export_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}


// 取消导出 PWM 通道
static bool pwm_unexport(PWM* pwm) {
    char unexport_path[100];
    int fd;
    char num_str[10];
    int_to_str(num_str, pwm->channel_num);
    snprintf(unexport_path, sizeof(unexport_path), UNEXPORT_PATH_FORMAT, pwm->chip_num);
    fd = open(unexport_path, O_WRONLY); // 移除 O_TRUNC 标志
    if (fd == -1) {
        perror("pwm_unexport: open unexport_path failed");
        return false;
    }
    if (write(fd, num_str, strlen(num_str)) == -1) {
        perror("pwm_unexport: write unexport_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}


// 获取 PWM 通道的路径
static void pwm_get_path(PWM* pwm, char* path, size_t size) {
    snprintf(path, size, PWM_PATH_FORMAT, pwm->chip_num, pwm->channel_num);
}


// 初始化 PWM 结构体
PWM* pwm_init(int chip_num, int channel_num) {
    PWM* pwm = (PWM*)malloc(sizeof(PWM));
    if (pwm == NULL) {
        perror("pwm_init: malloc failed");
        return NULL;
    }
    pwm->chip_num = chip_num;
    pwm->channel_num = channel_num;
    if (!pwm_export(pwm)) {
        free(pwm);
        return NULL;
    }
    return pwm;
}


// 设置 PWM 的频率
bool pwm_set_frequency(PWM* pwm, unsigned int frequency) {
    char pwm_path[100];
    char period_path[100];
    int fd;
    char period_str[30];
    unsigned long long period_ns = 1000000000ULL / frequency; // 频率转换为周期（纳秒）
    uint_to_str(period_str, period_ns);
    pwm_get_path(pwm, pwm_path, sizeof(pwm_path));
    snprintf(period_path, sizeof(period_path), PERIOD_PATH_FORMAT, pwm_path);
    fd = open(period_path, O_WRONLY); 
    if (fd == -1) {
        perror("pwm_set_frequency: open period_path failed");
        return false;
    }
    if (write(fd, period_str, strlen(period_str)) == -1) {
        perror("pwm_set_frequency: write period_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}


// 设置 PWM 的占空比
// ... existing code ...

bool pwm_set_duty_cycle(PWM* pwm, float duty_cycle) {
    char pwm_path[100];
    char duty_cycle_path[100];
    char period_path[100];
    char period_str[30];
    int fd;
    unsigned long long period_ns;
    char duty_cycle_str[30];
    
    // 获取PWM路径
    pwm_get_path(pwm, pwm_path, sizeof(pwm_path));
    
    // 读取当前周期
    snprintf(period_path, sizeof(period_path), PERIOD_PATH_FORMAT, pwm_path);
    fd = open(period_path, O_RDONLY);
    if (fd == -1) {
        perror("pwm_set_duty_cycle: open period_path failed");
        return false;
    }
    if (read(fd, period_str, sizeof(period_str)) == -1) {
        perror("pwm_set_duty_cycle: read period_path failed");
        close(fd);
        return false;
    }
    close(fd);
    
    // 将周期字符串转换为数值
    period_ns = strtoull(period_str, NULL, 10);
    
    // 计算低电平时间 = 周期 - (占空比 * 周期)
    
    unsigned long long low_time_ns = period_ns - (unsigned long long)(duty_cycle * period_ns);
    //printf("%lld,%f\n",low_time_ns,duty_cycle);
    snprintf(duty_cycle_str, sizeof(duty_cycle_str), "%llu", low_time_ns);
    
    // 写入低电平时间
    snprintf(duty_cycle_path, sizeof(duty_cycle_path), DUTY_CYCLE_PATH_FORMAT, pwm_path);
    fd = open(duty_cycle_path, O_WRONLY); 
    if (fd == -1) {
        perror("pwm_set_duty_cycle: open duty_cycle_path failed");
        return false;
    }
    if (write(fd, duty_cycle_str, strlen(duty_cycle_str)) == -1) {
        perror("pwm_set_duty_cycle: write duty_cycle_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}

// ... existing code ...



// 启用 PWM
bool pwm_enable(PWM* pwm) {
    char pwm_path[100];
    char enable_path[100];
    int fd;
    pwm_get_path(pwm, pwm_path, sizeof(pwm_path));
    snprintf(enable_path, sizeof(enable_path), ENABLE_PATH_FORMAT, pwm_path);
    fd = open(enable_path, O_WRONLY); 
    if (fd == -1) {
        perror("pwm_enable: open enable_path failed");
        return false;
    }
    if (write(fd, "1", 1) == -1) {
        perror("pwm_enable: write enable_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}


// 禁用 PWM
bool pwm_disable(PWM* pwm) {
    char pwm_path[100];
    char enable_path[100];
    int fd;
    pwm_get_path(pwm, pwm_path, sizeof(pwm_path));
    snprintf(enable_path, sizeof(enable_path), ENABLE_PATH_FORMAT, pwm_path);
    fd = open(enable_path, O_WRONLY); 
    if (fd == -1) {
        perror("pwm_disable: open enable_path failed");
        return false;
    }
    if (write(fd, "0", 1) == -1) {
        perror("pwm_disable: write enable_path failed");
        close(fd);
        return false;
    }
    close(fd);
    return true;
}


// 销毁 PWM 结构体释放资源
void pwm_destroy(PWM* pwm) {
    if (pwm == NULL) return;
    pwm_disable(pwm);
    if (!pwm_unexport(pwm)) {
        perror("pwm_destroy: pwm_unexport failed");
    }
    free(pwm);
}