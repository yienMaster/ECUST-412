/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
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
* 文件名称          zf_device_lora3a22
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-03-29       JKS            first version
********************************************************************************************************************/

#include "zf_device_lora3a22.h"

uint8   lora3a22_uart_data[LORA3A22_DATA_LEN]  = {0};               // 遥控器接收器原始数据

vuint8  lora3a22_finsh_flag = 0;                                    // 表示成功接收到一帧遥控器数据
vuint8  lora3a22_state_flag = 1;                                    // 遥控器状态(1表示正常，否则表示失控)
uint16  lora3a22_response_time = 0;

lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;

//--------------------------------  -----------------------------------------------------------------------------------
// 函数简介     lora3a22串口回调函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_uart_callback();
// 备注信息     此函数需要在串口接收中断内进行调用
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_uart_callback(void )
{
    static uint8 length = 0 ;
    uint8  parity_bit_sum  = 0, parity_bit  = 0;

    lora3a22_uart_data[length++] = uart_read_byte(LORA3A22_UART_INDEX);

    if((1 == length) && (LORA3A22_FRAME_STAR != lora3a22_uart_data[0]))
    {
        length =  0;
    }                                                             // 起始位判断

    if(LORA3A22_DATA_LEN <= length)                            	  // 数据长度判断
    {
        parity_bit = lora3a22_uart_data[1];
        lora3a22_uart_data[1] = 0;
        for(int  i = 0; i < LORA3A22_DATA_LEN; i ++)
        {
            parity_bit_sum += lora3a22_uart_data[i];
        }

        if (parity_bit_sum == parity_bit)                          // 和校验判断
        {
            lora3a22_finsh_flag = 1;
            lora3a22_state_flag = 1;
            lora3a22_response_time = 0;
            lora3a22_uart_data[1]= parity_bit;

            // 将接收到的数据拷贝到结构体中
            memcpy((uint8*)&lora3a22_uart_transfer, (uint8*)lora3a22_uart_data, \
            sizeof(lora3a22_uart_data));

        }
        else
        {
            lora3a22_finsh_flag = 0;
        }
        parity_bit_sum = 0;
        length = 0;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     lora3a22初始化函数
// 参数说明     void
// 返回参数     void
// 使用示例     lora3a22_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------

void lora3a22_init(void)
{
    uart_init(LORA3A22_UART_INDEX, LORA3A22_UART_BAUDRATE, LORA3A22_UART_TX_PIN, LORA3A22_UART_RX_PIN);

    uart_rx_interrupt(LORA3A22_UART_INDEX, 1);
    // 设置串口中断回调函数
    set_wireless_type(LORA3A22_UART, lora3a22_uart_callback);
}


