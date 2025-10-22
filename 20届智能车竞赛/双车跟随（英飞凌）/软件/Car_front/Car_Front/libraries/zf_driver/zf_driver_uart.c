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
* 文件名称          zf_driver_uart
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC364DP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-02       pudding            first version
* 2023-09-27       pudding            修改串口发送和接收的通信逻辑，将直接从底层读取或者写入数据，不再通过中断处理
* 2023-10-10       pudding            新增串口4-串口11
********************************************************************************************************************/

#include "IFXPORT.h"
#include "ifxAsclin_reg.h"
#include "ifxCpu_Irq.h"
#include "IFXASCLIN_CFG.h"
#include "SysSe/Bsp/Bsp.h"
#include "isr_config.h"
#include "zf_common_debug.h"
#include "zf_driver_uart.h"


// 创建串口handle变量
IfxAsclin_Asc uart0_handle;
IfxAsclin_Asc uart1_handle;
IfxAsclin_Asc uart2_handle;
IfxAsclin_Asc uart3_handle;
IfxAsclin_Asc uart4_handle;
IfxAsclin_Asc uart5_handle;
IfxAsclin_Asc uart6_handle;
IfxAsclin_Asc uart8_handle;
IfxAsclin_Asc uart9_handle;
IfxAsclin_Asc uart10_handle;
IfxAsclin_Asc uart11_handle;

// 创建一个ascConfig的结构体变量，只用于串口初始化
static IfxAsclin_Asc_Config uart_config;


// 创建串口缓存数组
static uint8 uart0_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart0_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart1_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart1_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart2_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart2_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart3_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart3_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart4_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart4_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart5_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart5_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart6_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart6_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart8_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart8_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart9_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart9_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart10_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart10_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart11_tx_buffer[1 + sizeof(Ifx_Fifo) + 8];
static uint8 uart11_rx_buffer[1 + sizeof(Ifx_Fifo) + 8];
//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口中断优先级设置
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 返回参数       void
// 使用示例       uart_set_interrupt_priority(UART_1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_set_interrupt_priority (uart_index_enum uartn)
{
    switch(uartn)
    {
        case UART_0:
        {
            uart_config.interrupt.txPriority    = UART0_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART0_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART0_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART0_INT_SERVICE;
        }break;
        case UART_1:
        {
            uart_config.interrupt.txPriority    = UART1_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART1_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART1_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART1_INT_SERVICE;
        }break;
        case UART_2:
        {
            uart_config.interrupt.txPriority    = UART2_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART2_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART2_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART2_INT_SERVICE;
        }break;
        case UART_3:
        {
            uart_config.interrupt.txPriority    = UART3_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART3_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART3_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART3_INT_SERVICE;
        }break;
        case UART_4:
        {
            uart_config.interrupt.txPriority    = UART4_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART4_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART4_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART4_INT_SERVICE;
        }break;
        case UART_5:
        {
            uart_config.interrupt.txPriority    = UART5_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART5_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART5_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART5_INT_SERVICE;
        }break;
        case UART_6:
        {
            uart_config.interrupt.txPriority    = UART6_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART6_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART6_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART6_INT_SERVICE;
        }break;
        case UART_8:
        {
            uart_config.interrupt.txPriority    = UART8_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART8_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART8_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART8_INT_SERVICE;
        }break;
        case UART_9:
        {
            uart_config.interrupt.txPriority    = UART9_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART9_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART9_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART9_INT_SERVICE;
        }break;
        case UART_10:
        {
            uart_config.interrupt.txPriority    = UART10_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART10_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART10_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART10_INT_SERVICE;
        }break;
        case UART_11:
        {
            uart_config.interrupt.txPriority    = UART11_TX_INT_PRIO;
            uart_config.interrupt.rxPriority    = UART11_RX_INT_PRIO;
            uart_config.interrupt.erPriority    = UART11_ER_INT_PRIO;
            uart_config.interrupt.typeOfService = UART11_INT_SERVICE;
        }break;
        default: zf_assert(FALSE);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       设置串口缓冲区
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 返回参数       void
// 使用示例       uart_set_buffer(UART_1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_set_buffer (uart_index_enum uartn)
{
    switch(uartn)
    {
        case UART_0:
        {
            uart_config.txBuffer     = &uart0_tx_buffer;
            uart_config.rxBuffer     = &uart0_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_1:
        {
            uart_config.txBuffer     = &uart1_tx_buffer;
            uart_config.rxBuffer     = &uart1_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_2:
        {
            uart_config.txBuffer     = &uart2_tx_buffer;
            uart_config.rxBuffer     = &uart2_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_3:
        {
            uart_config.txBuffer     = &uart3_tx_buffer;
            uart_config.rxBuffer     = &uart3_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_4:
        {
            uart_config.txBuffer     = &uart4_tx_buffer;
            uart_config.rxBuffer     = &uart4_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_5:
        {
            uart_config.txBuffer     = &uart5_tx_buffer;
            uart_config.rxBuffer     = &uart5_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_6:
        {
            uart_config.txBuffer     = &uart6_tx_buffer;
            uart_config.rxBuffer     = &uart6_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_8:
        {
            uart_config.txBuffer     = &uart8_tx_buffer;
            uart_config.rxBuffer     = &uart8_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_9:
        {
            uart_config.txBuffer     = &uart9_tx_buffer;
            uart_config.rxBuffer     = &uart9_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_10:
        {
            uart_config.txBuffer     = &uart10_tx_buffer;
            uart_config.rxBuffer     = &uart10_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        case UART_11:
        {
            uart_config.txBuffer     = &uart11_tx_buffer;
            uart_config.rxBuffer     = &uart11_rx_buffer;
            uart_config.txBufferSize = 1;
            uart_config.rxBufferSize = 1;
        }break;
        default: zf_assert(FALSE);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       获取串口中断配置信息
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 返回参数       void
// 使用示例       uart_get_handle(UART_1);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
IfxAsclin_Asc* uart_get_handle (uart_index_enum uartn)
{
    IfxAsclin_Asc* uart_handle = NULL;
    switch(uartn)
    {
        case UART_0:  uart_handle = &uart0_handle;  break;
        case UART_1:  uart_handle = &uart1_handle;  break;
        case UART_2:  uart_handle = &uart2_handle;  break;
        case UART_3:  uart_handle = &uart3_handle;  break;
        case UART_4:  uart_handle = &uart4_handle;  break;
        case UART_5:  uart_handle = &uart5_handle;  break;
        case UART_6:  uart_handle = &uart6_handle;  break;
        case UART_8:  uart_handle = &uart8_handle;  break;
        case UART_9:  uart_handle = &uart9_handle;  break;
        case UART_10: uart_handle = &uart10_handle; break;
        case UART_11: uart_handle = &uart11_handle; break;
        default: zf_assert(FALSE);
    }
    return uart_handle;
}


void uart_mux (uart_index_enum uartn, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin, uint32 *set_tx_pin, uint32 *set_rx_pin)
{
    switch(uartn)
    {
        case UART_0:
        {
            if     (UART0_TX_P14_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P14_0_OUT;
            else if(UART0_TX_P14_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P14_1_OUT;
            else if(UART0_TX_P15_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P15_2_OUT;
            else if(UART0_TX_P15_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin0_TX_P15_3_OUT;
            else zf_assert(FALSE);

            if     (UART0_RX_P14_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin0_RXA_P14_1_IN;
            else if(UART0_RX_P15_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin0_RXB_P15_3_IN;
            else zf_assert(FALSE);

        }break;
        case UART_1:
        {
            if     (UART1_TX_P02_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P02_2_OUT;
            else if(UART1_TX_P11_12 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P11_12_OUT;
            else if(UART1_TX_P15_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_0_OUT;
            else if(UART1_TX_P15_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_1_OUT;
            else if(UART1_TX_P15_4  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_4_OUT;
            else if(UART1_TX_P15_5  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P15_5_OUT;
            else if(UART1_TX_P20_10 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P20_10_OUT;
            else if(UART1_TX_P33_12 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P33_12_OUT;
            else if(UART1_TX_P33_13 == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin1_TX_P33_13_OUT;
            else zf_assert(FALSE);

            if     (UART1_RX_P15_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXA_P15_1_IN;
            else if(UART1_RX_P15_5  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXB_P15_5_IN;
            else if(UART1_RX_P20_9  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXC_P20_9_IN;
            else if(UART1_RX_P11_10 == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXE_P11_10_IN;
            else if(UART1_RX_P33_13 == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXF_P33_13_IN;
            else if(UART1_RX_P02_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin1_RXG_P02_3_IN;
            else zf_assert(FALSE);

        }break;
        case UART_2:
        {
            if     (UART2_TX_P02_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P02_0_OUT;
            else if(UART2_TX_P10_5  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P10_5_OUT;
            else if(UART2_TX_P14_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P14_2_OUT;
            else if(UART2_TX_P14_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P14_3_OUT;
            else if(UART2_TX_P33_8  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P33_8_OUT;
            else if(UART2_TX_P33_9  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin2_TX_P33_9_OUT;
            else zf_assert(FALSE);

            if     (UART2_RX_P14_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXA_P14_3_IN;
            else if(UART2_RX_P02_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXB_P02_1_IN;
            else if(UART2_RX_P10_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXD_P10_6_IN;
            else if(UART2_RX_P33_8  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXE_P33_8_IN;
            else if(UART2_RX_P02_0  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin2_RXG_P02_0_IN;

            else zf_assert(FALSE);

        }break;
        case UART_3:
        {
            if     (UART3_TX_P00_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P00_0_OUT;
            else if(UART3_TX_P00_1  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P00_1_OUT;
            else if(UART3_TX_P15_6  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P15_6_OUT;
            else if(UART3_TX_P15_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P15_7_OUT;
            else if(UART3_TX_P20_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P20_0_OUT;
            else if(UART3_TX_P20_3  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P20_3_OUT;
            else if(UART3_TX_P21_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin3_TX_P21_7_OUT;
            else zf_assert(FALSE);

            if     (UART3_RX_P15_7  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXA_P15_7_IN;
            else if(UART3_RX_P20_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXC_P20_3_IN;
            else if(UART3_RX_P00_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXE_P00_1_IN;
            else if(UART3_RX_P21_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin3_RXF_P21_6_IN;
            else zf_assert(FALSE);

        }break;
        case UART_4:
        {
            if     (UART4_TX_P00_9  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin4_TX_P00_9_OUT;
            else zf_assert(FALSE);

            if     (UART4_RX_P00_12  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin4_RXA_P00_12_IN;
            else zf_assert(FALSE);

        }break;
        case UART_5:
        {
            if     (UART5_TX_P00_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin5_TX_P00_7_OUT;
            else if(UART5_TX_P22_2  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin5_TX_P22_2_OUT;
            else zf_assert(FALSE);

            if     (UART5_RX_P00_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin5_RXA_P00_6_IN;
            else if(UART5_RX_P22_3  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin5_RXC_P22_3_IN;
            else if(UART5_RX_P33_4  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin5_RXB_P33_4_IN;
            else zf_assert(FALSE);

        }break;
        case UART_6:
        {
            if     (UART6_TX_P22_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin6_TX_P22_0_OUT;
            else zf_assert(FALSE);

            if     (UART6_RX_P22_0  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin6_RXE_P22_0_IN;
            else if(UART6_RX_P23_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin6_RXF_P23_1_IN;
            else zf_assert(FALSE);

        }break;
        case UART_8:
        {
            if     (UART8_TX_P33_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin8_TX_P33_7_OUT;
            else zf_assert(FALSE);

            if     (UART8_RX_P33_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin8_RXD_P33_6_IN;
            else zf_assert(FALSE);

        }break;
        case UART_9:
        {
            if     (UART9_TX_P20_7  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin9_TX_P20_7_OUT;
            else zf_assert(FALSE);

            if     (UART9_RX_P20_6  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin9_RXE_P20_6_IN;
            else zf_assert(FALSE);

        }break;
        case UART_10:
        {
            if     (UART10_TX_P00_8  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin10_TX_P00_8_OUT;
            else if(UART10_TX_P13_0  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin10_TX_P13_0_OUT;
            else zf_assert(FALSE);

            if     (UART10_RX_P00_4  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin10_RXA_P00_4_IN;
            else if(UART10_RX_P00_8  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin10_RXB_P00_8_IN;
            else if(UART10_RX_P13_0  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin10_RXC_P13_0_IN;
            else if(UART10_RX_P13_1  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin10_RXD_P13_1_IN;
            else zf_assert(FALSE);

        }break;

        case UART_11:
        {
            if     (UART11_TX_P21_5  == tx_pin)  *set_tx_pin = (uint32)&IfxAsclin11_TX_P21_5_OUT;
            else zf_assert(FALSE);

            if     (UART11_RX_P21_2  == rx_pin)  *set_rx_pin = (uint32)&IfxAsclin11_RXE_P21_2_IN;
            else zf_assert(FALSE);

        }break;
        default:break;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口发送写入
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       dat             需要发送的字节
// 返回参数       void
// 使用示例       uart_write_byte(UART_1, 0xA5);                    // 往串口1的发送缓冲区写入0xA5，写入后仍然会发送数据，但是会减少CPU在串口的执行时
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_write_byte (uart_index_enum uart_n, const uint8 dat)
{
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);

    while(IfxAsclin_getTxFifoFillLevel(uart_handle->asclin) != 0);
    IfxAsclin_write8(uart_handle->asclin, &dat, 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口发送数组
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       *buff           要发送的数组地址
// 参数说明       len             发送长度
// 返回参数       void
// 使用示例       uart_write_buffer(UART_1, &a[0], 5);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_write_buffer (uart_index_enum uart_n, const uint8 *buff, uint32 len)
{
    while(len)
    {
        uart_write_byte(uart_n, *buff);
        len--;
        buff++;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口发送字符串
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       *str            要发送的字符串地址
// 返回参数       void
// 使用示例       uart_write_string(UART_1, "seekfree");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_write_string (uart_index_enum uart_n, const char *str)
{
    while(*str)
    {
        uart_write_byte(uart_n, *str++);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       读取串口接收的数据（whlie等待）
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       *dat            接收数据的地址
// 返回参数       uint8           接收的数据
// 使用示例       uint8 dat = uart_read_byte(UART_1);             // 接收 UART_1 数据  存在在 dat 变量里
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_read_byte (uart_index_enum uart_n)
{

    uint8 return_num = 0;
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);
    while(IfxAsclin_getRxFifoFillLevel(uart_handle->asclin) == 0);
    IfxAsclin_read8(uart_handle->asclin, &return_num, 1);

    return return_num;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       读取串口接收的数据（查询接收）
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       *dat            接收数据的地址
// 返回参数       uint8           1：接收成功   0：未接收到数据
// 使用示例       uint8 dat; uart_query_byte(UART_1, &dat);       // 接收 UART_1 数据  存在在 dat 变量里
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 uart_query_byte (uart_index_enum uart_n, uint8 *dat)
{
    uint8 return_num = 0;
    IfxAsclin_Asc* uart_handle;
    uart_handle = uart_get_handle(uart_n);
    if(IfxAsclin_getRxFifoFillLevel(uart_handle->asclin) > 0)
    {
        IfxAsclin_read8(uart_handle->asclin, dat, 1);
        return_num = 1;
    }
    return return_num;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口发送中断设置
// 参数说明       uart_n           串口模块号
// 参数说明       status          1：打开中断   0：关闭中断
// 返回参数       void
// 使用示例       uart_tx_interrupt(UART_1, 1);                   // 打开串口1发送中断
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_tx_interrupt (uart_index_enum uart_n, uint32 status)
{
    Ifx_ASCLIN      *asclinSFR = uart_config.asclin;
    volatile Ifx_SRC_SRCR *src;
    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle); // 初始化化配置结构体
    src = IfxAsclin_getSrcPointerTx(asclinSFR);
    IfxAsclin_enableTxFifoFillLevelFlag(asclinSFR, (boolean)status);
    if(status)
    {
        IfxSrc_enable(src);
    }
    else
    {
        IfxSrc_disable(src);
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介       串口接收中断设置
// 参数说明       uart_n           串口模块号
// 参数说明       status          1：打开中断   0：关闭中断
// 返回参数       void
// 使用示例       uart_rx_interrupt(UART_1, 1);                   // 打开串口1接收中断
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt (uart_index_enum uart_n, uint32 status)
{
    Ifx_ASCLIN      *asclinSFR = uart_config.asclin;
    volatile Ifx_SRC_SRCR *src;
    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle); // 初始化化配置结构体
    src = IfxAsclin_getSrcPointerRx(asclinSFR);
    IfxAsclin_enableRxFifoFillLevelFlag(asclinSFR, (boolean)status);
    if(status)
    {
        IfxSrc_enable(src);
    }
    else
    {
        IfxSrc_disable(src);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     sbus初始化
// 参数说明     uartn       串口通道(UART_0,UART_1,UART_2,UART_3)
// 参数说明     baud        串口波特率
// 参数说明     tx_pin      串口发送引脚号
// 参数说明     rx_pin      串口接收引脚号
// 返回参数     void
// 使用示例     uart_sbus_init(UART_2, 100000, UART2_TX_P10_5, UART2_RX_P10_6);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_sbus_init (uart_index_enum uartn, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{

    boolean interrupt_state = disableInterrupts();

    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uartn);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle);   // 初始化化配置结构体

    uart_set_buffer(uartn);                                 // 设置缓冲区

    uart_set_interrupt_priority(uartn);                     // 设置中断优先级

    uart_config.clockSource           = IfxAsclin_ClockSource_ascFastClock;     // 使用高速时钟 最大波特率6.25M
    uart_config.baudrate.prescaler    = 4;
    uart_config.baudrate.baudrate     = (float32)baud;
    uart_config.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    uart_config.frame.stopBit         =  IfxAsclin_StopBit_2;                    //停止位
    uart_config.frame.parityType      = IfxAsclin_ParityType_even;               //偶校验
    uart_config.frame.dataLength      = IfxAsclin_DataLength_8;
    uart_config.frame.parityBit       = TRUE;                                   //启动校验

    IfxAsclin_Asc_Pins pins;                                                    // 设置引脚
    pins.cts = NULL;
    pins.rts = NULL;
    uart_mux(uartn, tx_pin, rx_pin, (uint32 *)&pins.tx, (uint32 *)&pins.rx);
    pins.rxMode = IfxPort_InputMode_pullUp;
    pins.txMode = IfxPort_OutputMode_pushPull;
    pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    uart_config.pins = &pins;

    IfxAsclin_Asc_initModule(uart_get_handle(uartn), &uart_config);
    uart_rx_interrupt(uartn, 1);
    uart_tx_interrupt(uartn, 0);
    restoreInterrupts(interrupt_state);

}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      串口初始化
//  参数说明      uartn           串口模块号(UART_0,UART_1,UART_2,UART_3)
//  参数说明      baud            串口波特率
//  参数说明      tx_pin          串口发送引脚
//  参数说明      rx_pin          串口接收引脚
//  返回参数      uint32          实际波特率
//  使用示例      uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);       // 初始化串口0 波特率115200 发送引脚使用P14_0 接收引脚使用P14_1
//  备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_init (uart_index_enum uart_n, uint32 baud, uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    boolean interrupt_state = disableInterrupts();

    volatile Ifx_ASCLIN *moudle = IfxAsclin_getAddress((IfxAsclin_Index)uart_n);

    IfxAsclin_Asc_initModuleConfig(&uart_config, moudle);   // 初始化化配置结构体

    uart_set_buffer(uart_n);                                // 设置缓冲区

    uart_set_interrupt_priority(uart_n);                    // 设置中断优先级

    uart_config.clockSource           = IfxAsclin_ClockSource_ascFastClock;     // 使用高速时钟 最大波特率6.25M
    uart_config.baudrate.prescaler    = 4;
    uart_config.baudrate.baudrate     = (float32)baud;
    uart_config.baudrate.oversampling = IfxAsclin_OversamplingFactor_8;

    IfxAsclin_Asc_Pins pins;                                // 设置引脚
    pins.cts = NULL;
    pins.rts = NULL;
    uart_mux(uart_n, tx_pin, rx_pin, (uint32 *)&pins.tx, (uint32 *)&pins.rx);
    pins.rxMode = IfxPort_InputMode_pullUp;
    pins.txMode = IfxPort_OutputMode_pushPull;
    pins.pinDriver = IfxPort_PadDriver_cmosAutomotiveSpeed1;
    uart_config.pins = &pins;

    IfxAsclin_Asc_initModule(uart_get_handle(uart_n), &uart_config);
    uart_rx_interrupt(uart_n, 0);
    uart_tx_interrupt(uart_n, 0);
    restoreInterrupts(interrupt_state);
}
