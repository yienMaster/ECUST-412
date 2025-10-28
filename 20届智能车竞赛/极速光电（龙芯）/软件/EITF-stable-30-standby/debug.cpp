

#include "debug.h"


void tcp_debug(const char *server_ip,const int port)
{
    
    
    struct sockaddr_in serv_addr;

    // 创建套接字
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "创建套接字失败" << std::endl;
        raise(SIGINT);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port); // 设置服务器端口号

    // 将服务器地址转换为二进制格式
    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) { 
        std::cerr << "地址转换失败" << std::endl;
        raise(SIGINT);
    }

    // 连接到服务器
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "连接失败" << std::endl;
        raise(SIGINT);
    }



    return;
}



//非自适应PID
/**/
struct PIDController {
    // p < 0.013   <=0.005   <0.007   <0.006
    //i  <0.2   <0.1   <0.05   <0.005
    /*
    float Kp = 0.005f;   // 比例系数（需调试）
    float Ki = 0.00f; // 积分系数（需调试） 
    float Kd = 0.00f;  // 微分系数（需调试）
    */

    //速度20   80.0
    float Kp = 0.245f;   // 比例系数（需调试）
    float Ki = 0.00f; // 积分系数（需调试） 
    float Kd = 0.000f;  // 微分系数（需调试）
    //0.003   0.024 -0.25
    

    //已调试出的合适   PID_servo 综合输出那里/80.0f   速度 10
    // P=0.02   K=0.015 D=0.0013
    // 0.025  0.01  0.0013
    //0.03 0.01 0.0013轻微震荡
    // 运行参数
    float integral = 0.0f;      // 积分累计
    float prev_error = 0.0f;    // 上一次误差
    float prev_time = 0.0f;     // 上一次时间戳
    float output = 0.45f;       // 初始化直行PWM
    
    // 系统参数
    const float PWM_MIN = 0.41f; // 最大右转
    const float PWM_MAX = 0.49f; // 最大左转
    const float PWM_CENTER = 0.45f;


     // 新增前馈参数
     float Kf = 0.0f;          // 前馈增益系数（需调试）
     float steering_rate = 0.0f; // 舵机角速度估计
     float prev_steering = 0.45f; // 前次舵机指令
     float rate_filter = 0.2f;   // 角速度低通滤波系数（0.1-0.3）

};
