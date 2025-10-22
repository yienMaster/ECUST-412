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

#include "zf_device_type.h"
#include "zf_driver_uart.h"
#include "zf_driver_timer.h"
#include "zf_device_uart_receiver.h"

uart_receiver_struct    uart_receiver ;             // 串口接收机通道数据与状态

uint8   uart_receiver_data[REV_DATA_LEN]  = {0};    // 接收器原始数据

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     获取两次调用uart_receiver_interval_time函数时间间隔
//  参数说明     void
//  返回参数     uint32            返回从开始到现在的时间(单位1us)
//  使用示例     uint32 time = uart_receiver_interval_time();
//  备注信息     内部调用
//-------------------------------------------------------------------------------------------------------------------
static uint32 uart_receiver_interval_time (void)
{
    static uint32 time_last = 0;
    uint32 time, interval_time;
    uint32 stm_clk;

    stm_clk = IfxStm_getFrequency(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));

    time = IfxStm_getLower(IfxStm_getAddress((IfxStm_Index)(IfxCpu_getCoreId())));
    interval_time = time - time_last;
    time_last = time;
    interval_time = (uint32)((uint64)interval_time * 1000000 / stm_clk);

    return interval_time;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     SBUS数据解析
// 参数说明     *remote_data    接收遥控器数据的地址
// 参数说明     *bufer          原始数据
// 返回参数     void
// 使用示例
// 备注信息     对sbus数据解析进行解析
//-------------------------------------------------------------------------------------------------------------------
static void uart_receiver_analysis (uart_receiver_struct *remote_data,uint8 * buffer)
{
    uint8 num = 0;
    remote_data->channel[num++] = (buffer[1] |buffer[ 2] << 8 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[2] >> 3 | buffer[3] << 5 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[5] >> 1 | buffer[6] << 7 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[6] >> 4 | buffer[7] << 4 ) & 0x07FF;
    remote_data->channel[num++] = (buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9 ) & 0x07FF;
    remote_data->state = (SBUS_NORMAL_STATE == buffer[23]) ? 1 : 0;
    uart_receiver.finsh_flag  = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口接收机模块 串口中断回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     uart_receiver_callback();
// 备注信息     该函数在 ISR 文件 串口中断程序通过uart_receiver_uart_handler函数指针被调用
//-------------------------------------------------------------------------------------------------------------------
void uart_receiver_callback(void)
{
    static vuint8 length = 0;

    if (uart_receiver_interval_time() > 3000)
    {
        length = 0;
    }
    uart_receiver_data[length++] = uart_read_byte(UART_RECEVIER_UART_INDEX);
    if  ( (REV_DATA_LEN  == length)                                                                    // 如果帧长、帧头、帧尾满足协议
        && (FRAME_STAR   == uart_receiver_data[0])
        && (FRAME_END    == uart_receiver_data[24]))
    {
        uart_receiver_analysis(&uart_receiver, uart_receiver_data);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     串口接收机初始化
// 参数说明     void
// 返回参数     void
// 使用示例     uart_receiver_init();
//-------------------------------------------------------------------------------------------------------------------
void uart_receiver_init(void)
{
    uart_sbus_init(UART_RECEVIER_UART_INDEX, SBUS_UART_BAUDRATE, UART_RECEVIER_TX_PIN, UART_RECEVIER_RX_PIN);

    // 设置串口中断回调函数
    set_wireless_type(RECEIVER_UART, uart_receiver_callback);
}
