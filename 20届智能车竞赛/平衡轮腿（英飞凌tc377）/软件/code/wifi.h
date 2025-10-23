#ifndef CODE_WIFIS_H_
#define CODE_WIFI_H_

#include "zf_common_headfile.h"


#define WIFI_SSID_TEST          "xxx"        //名称
#define WIFI_PASSWORD_TEST      "118837235"  //密码
// 如果需要连接的WIFI 没有密码则需要将 这里 替换为 NULL

#define WIFI_TARGET_IP      "192.168.1.180"       // 连接目标的 IP
#define WIFI_TARGET_PORT    "8086"                 // 连接目标的端口
#define WIFI_LOCAL_PORT     "6666"                 // 本机的端口 0：随机  可设置范围2048-65535  默认 6666

//WiFi初始化
void wifi_init(void);

//上位机串口TCP连接
void TCP_client_init(void);
//逐飞上位机TCP连接
void TCP_seekfree_init(void);

//wifi串口发送
//wifi串口发送整数
void wifi_send_int(int a);
//wifi串口发送浮点数
void wifi_send_float(float a);
//wifi串口发送字符
void wifi_send_char(char wifi_data_buffer[]);
//wifi串口接收
void wifi_get(uint8 wifi_spi_get_data_buffer[]);


// 逐飞助手初始化
void seekfree_init(void);
//逐飞助手波形设置
void seekfree_oscilloscope_send(void);
//逐飞助手摄像头发送图像
void seekfree_camera_send(void);

#endif /* CODE_PID_H_ */
