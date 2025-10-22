/*********************************************************************************************************************
* TC364 Opensourec Library 即（TC364 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC364 开源库的一部分
*
* TC364 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_wifi_spi
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC364DP
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-01-18        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
*                   INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
*                   CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
*                   MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
*                   SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
*                   MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
*                   5V                  5V 电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/

#ifndef _zf_device_wifi_spi_h
#define _zf_device_wifi_spi_h

#include "zf_common_typedef.h"

          
#define WIFI_SPI_INDEX              (SPI_3             )        // 定义使用的SPI号
#define WIFI_SPI_SPEED              (30 * 1000 * 1000  )        // 硬件 SPI 速率
#define WIFI_SPI_SCK_PIN            (SPI3_SCLK_P22_3   )        // 定义SPI_SCK引脚
#define WIFI_SPI_MOSI_PIN           (SPI3_MOSI_P22_0   )        // 定义SPI_MOSI引脚
#define WIFI_SPI_MISO_PIN           (SPI3_MISO_P22_1   )        // 定义SPI_MISO引脚  IPS没有MISO引脚，但是这里任然需要定义，在spi的初始化时需要使用
#define WIFI_SPI_CS_PIN             (P22_2             )        // 定义SPI_CS引脚 采用软件CS引脚
#define WIFI_SPI_INT_PIN            (P15_8             )        // 定义握手引脚
#define WIFI_SPI_RST_PIN            (P23_1             )        // 定义复位引脚


#define WIFI_SPI_RECVIVE_FIFO_SIZE  (1024)                      // 接收FIFO大小
#define WIFI_SPI_READ_TRANSFER      (1)                         // 在调用wifi_spi_read_buffer 是否尝试发起SPI通讯来检测模块内是否有数据需要读取 1：发起SPI通讯 0：不发起SPI通讯，仅读取FIFO
                                                                // 如果应用程序中没有任何的地方调用发送函数，则WIFI_SPI_READ_TRANSFER必须设置为1
                                                                
#define WIFI_SPI_AUTO_CONNECT       (0)                         // 定义是否初始化时建立TCP或者UDP连接    0-不自动连接  1-自动连接TCP服务器  2-自动连接UDP

#if     (WIFI_SPI_AUTO_CONNECT > 2)    
#error "WIFI_SPI_AUTO_CONNECT 的值只能为 [0,1,2]"
#else   
#define WIFI_SPI_TARGET_IP          "192.168.137.1"              // 连接目标的 IP
#define WIFI_SPI_TARGET_PORT        "8086"                      // 连接目标的端口
#define WIFI_SPI_LOCAL_PORT         "6666"                      // 本机的端口 0：随机  可设置范围2048-65535  默认 6666
#endif


#define WIFI_SPI_RECVIVE_SIZE       (32)                        // 每次SPI传输接收的字节数 不允许修改
#define WIFI_SPI_TRANSFER_SIZE      (4088)                      // 最大SPI传输接收的字节数 不允许修改



typedef enum
{
    // 主机发送的命令
    WIFI_SPI_INVALID1               = 0x00,                     // 无效数据包
    WIFI_SPI_RESET                  = 0x01,                     // 复位命令
    WIFI_SPI_DATA                   = 0x02,                     // 透传数据包
    WIFI_SPI_UDP_SEND               = 0x03,                     // UDP下立即发送命令,默认SPI接收数据后2MS未收到数据自动发送数据
    WIFI_SPI_CLOSE_SOCKET           = 0x04,                     // 断开连接
                
    WIFI_SPI_SET_WIFI_INFORMATION   = 0x10,                     // 设置WIFI信息命令
    WIFI_SPI_SET_SOCKET_INFORMATION = 0x11,                     // 设置SOCKET信息命令
                    
    WIFI_SPI_GET_VERSION            = 0x20,                     // 获取模块版本
    WIFI_SPI_GET_MAC_ADDR           = 0x21,                     // 获取模块MAC地址
    WIFI_SPI_GET_IP_ADDR            = 0x22,                     // 获取模块IP地址
                    
    // 从机回传的命令
    WIFI_SPI_REPLY_OK               = 0x80,                     // 从机应答的正确命令
    WIFI_SPI_REPLY_ERROR            = 0x81,                     // 从机应答的错误命令
                    
    WIFI_SPI_REPLY_DATA_START       = 0x90,                     // 从机回传的数据包，并且还有数据需要主机读取
    WIFI_SPI_REPLY_DATA_END         = 0x91,                     // 从机回传的数据包，数据已读取完毕
                    
    WIFI_SPI_REPLY_VERSION          = 0xA0,                     // 从机回复固件版本
    WIFI_SPI_REPLY_MAC_ADDR         = 0xA1,                     // 从机回复本机MAC地址等信息
    WIFI_SPI_REPLY_IP_ADDR          = 0xA2,                     // 从机回复本机IP地址、端口号
    WIFI_SPI_INVALID2               = 0xFF                      // 无效数据包
}wifi_spi_packets_command_enum;             
                
typedef enum                
{               
    WIFI_SPI_IDLE,                                              // 模块空闲，可以进行SPI通讯
    WIFI_SPI_BUSY,                                              // 模块正忙，不可进行SPI通讯
}wifi_spi_state_enum;               
                
                
typedef struct              
{               
    uint8   command;                                            // 命令字
    uint8   reserve;                                            // 保留
    uint16  length;                                             // 包有效长度
}wifi_spi_head_struct;              
                
                
typedef struct              
{               
    wifi_spi_head_struct  head;                                 // 帧头
    uint8 buffer[WIFI_SPI_RECVIVE_SIZE];                        // 缓冲区
}wifi_spi_packets_struct;               
                
                
extern char wifi_spi_version[12];                               // 固件版本         字符串
extern char wifi_spi_mac_addr[20];                              // 模块MAC地址      字符串
extern char wifi_spi_ip_addr_port[25];                          // IP地址与端口号   字符串

uint8  wifi_spi_wifi_connect        (char *wifi_ssid, char *pass_word);
uint8  wifi_spi_socket_connect      (char *transport_type, char *ip_addr, char *port, char *local_port);
uint8  wifi_spi_socket_disconnect   (void);
uint8  wifi_spi_udp_send_now        (void);
uint32 wifi_spi_send_buffer         (const uint8 *buff, uint32 length);
uint32 wifi_spi_read_buffer         (uint8 *buffer, uint32 length);

uint8  wifi_spi_init                (char *wifi_ssid, char *pass_word);

#endif

