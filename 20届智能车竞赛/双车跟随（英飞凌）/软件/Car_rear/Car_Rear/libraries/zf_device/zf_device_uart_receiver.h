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
* 文件名称          zf_device_uart_receiver
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC364DP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-04       JKS                first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   TXD               查看 zf_device_uart_receiver.h 中 UART_RECEVIER_RX_PIN 宏定义
*                   GND               电源地
*                   5V                5V电源
*                   ------------------------------------
********************************************************************************************************************/
#ifndef _zf_device_uart_receiver_h
#define _zf_device_uart_receiver_h

#include "zf_common_typedef.h"

#define UART_RECEVIER_UART_INDEX            UART_2              // 定义串口接收机使用的串口
#define UART_RECEVIER_TX_PIN                UART2_TX_P10_5      // 遥控器接收机没有这个引脚，仅用于串口初始化时占位使用
#define UART_RECEVIER_RX_PIN                UART2_RX_P10_6      // 串口接收机的TX引脚 连接单片机的RX引脚
#define SBUS_UART_BAUDRATE                  (100000)            // 指定 SBUS 串口所使用的的串口波特率 (用户不可修改)
#define UART_RECEVIER_CHANNEL_NUM           ( 6 )               // 定义遥控器通道数量 (用户不可修改)

#define REV_DATA_LEN                        ( 25   )            // SBUS帧长
#define FRAME_STAR                          ( 0X0F )            // 帧头信息
#define FRAME_END                           ( 0X00 )            // 帧尾信息
#define SBUS_NORMAL_STATE                   ( 0X03 )            // 正常状态
#define SBUS_ABNORMAL_STATE                 ( 0X0F )            // 失控状态

typedef struct
{
    uint16 channel[UART_RECEVIER_CHANNEL_NUM];                  // CH1-CH6通道数据
    uint8  state;                                               // 遥控器状态(1表示正常，否则表示失控)
    uint8  finsh_flag;                                          // 1：表示成功接收到一帧遥控器数据
}uart_receiver_struct;

extern uart_receiver_struct uart_receiver;                      // 串口接收机通道数据与状态

void uart_receiver_init(void);

#endif
