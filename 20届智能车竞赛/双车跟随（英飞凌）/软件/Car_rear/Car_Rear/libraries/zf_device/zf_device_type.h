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
* 文件名称          zf_device_type
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC364DP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-02       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
* 2023-05-26       pudding            新增SPI WIFI 中断回调指针
********************************************************************************************************************/

#ifndef _zf_device_type_h_
#define _zf_device_type_h_

#include "zf_common_debug.h"

//==============================================定义 外设 参数结构体==================================================
typedef enum
{
    NO_CAMERE = 0,                                                              // 无摄像头
    CAMERA_BIN_IIC,                                                             // 小钻风 IIC 版本
    CAMERA_BIN_UART,                                                            // 小钻风 UART 版本
    CAMERA_GRAYSCALE,                                                           // 总钻风
    CAMERA_COLOR,                                                               // 凌瞳
}camera_type_enum;

typedef enum
{
    NO_WIRELESS = 0,                                                            // 无设备
    WIRELESS_UART,                                                              // 无线串口
    BLE6A20,                                                                    // 蓝牙串口
    BLUETOOTH_CH9141,                                                           // 蓝牙 CH9141
    WIFI_UART,                                                                  // 串口 WiFi
    RECEIVER_UART,                                                              // 枪式遥控器
}wireless_type_enum;

typedef enum
{
    NO_TOF = 0,                                                                 // 无设备
    TOF_DL1A,                                                                   // DL1A
    TOF_DL1B,                                                                   // DL1B
}tof_type_enum;
//==============================================定义 外设 参数结构体==================================================


//===========================================声明 回调函数指针及外设 类型==============================================
typedef void (*callback_function)(void);

extern wireless_type_enum wireless_type;
extern callback_function wireless_module_uart_handler;                          // 无线串口接收中断函数指针，根据初始化时设置的函数进行跳转

extern camera_type_enum camera_type;
extern callback_function camera_dma_handler;                                    // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
extern callback_function camera_vsync_handler;                                  // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
extern callback_function camera_uart_handler;                                   // 串口通讯中断函数指针，根据初始化时设置的函数进行跳转
extern callback_function wireless_module_spi_handler;                           // WIFI SPI GPIO中断函数指针，根据初始化时设置的函数进行跳转

extern tof_type_enum     tof_type;                                              // ToF 模块 类型
extern callback_function tof_module_exti_handler;                               // ToF 模块 INT 更新中断
//===========================================声明 回调函数指针及外设 类型==============================================


//=============================================声明 中断回调 基础函数================================================
void   set_camera_type          (camera_type_enum type_set, callback_function vsync_callback, callback_function dma_callback, callback_function uart_callback);
void   set_wireless_type        (wireless_type_enum type_set, callback_function wireless_callback);
void   set_tof_type             (tof_type_enum type_set, callback_function exti_callback);
//=============================================声明 中断回调 基础函数================================================

#endif
