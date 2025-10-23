/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
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
* 文件名称          zf_driver_delay
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
* 2023-11-28       pudding            修改普通延时逻辑为中断触发，若总中断关闭则使用寄存器循环读取延时
********************************************************************************************************************/

#include "IfxStm.h"
#include "IFXSTM_CFG.h"
#include "SysSe/Bsp/Bsp.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"


#define IFX_INTPRIO_STM0_SR0  220
#define IFX_INTPRIO_STM1_SR0  221
#define IFX_INTPRIO_STM2_SR0  222

static vuint8 stm0_isr_flag = 1;
static vuint8 stm1_isr_flag = 1;
static vuint8 stm2_isr_flag = 1;

IFX_INTERRUPT(stm0_isr, 0, IFX_INTPRIO_STM0_SR0)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxStm_clearCompareFlag(&MODULE_STM0, IfxStm_Comparator_0);
    stm0_isr_flag = 0;
}

IFX_INTERRUPT(stm1_isr, 1, IFX_INTPRIO_STM1_SR0)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxStm_clearCompareFlag(&MODULE_STM1, IfxStm_Comparator_0);
    stm1_isr_flag = 0;
}

IFX_INTERRUPT(stm2_isr, 2, IFX_INTPRIO_STM2_SR0)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    IfxStm_clearCompareFlag(&MODULE_STM2, IfxStm_Comparator_0);
    stm2_isr_flag = 0;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介      system延时函数
//  参数说明      time            延时一轮的时间（单位为纳秒，可设置范围0-20000000）
//  返回参数      void
//  使用示例      无需用户调用，用户请使用h文件中的宏定义 // 100M时钟 最小单位为 10ns
//-------------------------------------------------------------------------------------------------------------------
void system_delay_10ns (uint32 time)
{
    uint32 interrupt_global_state;

    IfxStm_Index stm_index;

    stm_index = (IfxStm_Index)IfxCpu_getCoreId();

    if(areInterruptsEnabled() == 0)
    {
        IfxStm_waitTicks(IfxStm_getAddress(stm_index), time);
    }
    else
    {
        switch(stm_index)
        {
            case IfxStm_Index_0:
            {
                Ifx_STM *stm_sfr = &MODULE_STM0;
                stm0_isr_flag = 1;
                interrupt_global_state = interrupt_global_disable();     // 关闭全局中断
                stm_sfr->CMP[0].U = stm_sfr->TIM0.U + time;
                interrupt_global_enable(interrupt_global_state);         // 打开全局中断
                while(stm0_isr_flag);
            }break;
            case IfxStm_Index_1:
            {
                Ifx_STM *stm_sfr = &MODULE_STM1;
                stm1_isr_flag = 1;
                interrupt_global_state = interrupt_global_disable();     // 关闭全局中断
                stm_sfr->CMP[0].U = stm_sfr->TIM0.U + time;
                interrupt_global_enable(interrupt_global_state);         // 打开全局中断
                while(stm1_isr_flag);
            }break;
            case IfxStm_Index_2:
            {
                Ifx_STM *stm_sfr = &MODULE_STM2;
                stm2_isr_flag = 1;
                interrupt_global_state = interrupt_global_disable();     // 关闭全局中断
                stm_sfr->CMP[0].U = stm_sfr->TIM0.U + time;
                interrupt_global_enable(interrupt_global_state);         // 打开全局中断
                while(stm2_isr_flag);
            }break;
            case IfxStm_Index_none: break;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      system 微秒 延时函数(访问寄存器)
//  参数说明      time            延时时间  单位：us
//  返回参数      void
//  使用示例      system_delay_us_register(100);  // 延时100微秒
//-------------------------------------------------------------------------------------------------------------------
void system_delay_us_register (uint32 time)
{
    IfxStm_Index stm_index;

    stm_index = (IfxStm_Index)IfxCpu_getCoreId();

    IfxStm_waitTicks(IfxStm_getAddress(stm_index), time * 100);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      system 微秒 延时函数
//  参数说明      time            延时时间  单位：us
//  返回参数      void
//  使用示例      system_delay_us(100);  // 延时100微秒
//-------------------------------------------------------------------------------------------------------------------
void system_delay_us (uint32 time)
{
    if(time > 40000000)
    {
        while(time > 40000000)
        {
            system_delay_10ns(4000000000);
            time = time - 40000000;
        }
        system_delay_10ns(time * 100);
    }
    else
    {
        system_delay_10ns(time * 100);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      system 毫秒 延时函数
//  参数说明      time            延时时间  单位：ms
//  返回参数      void
//  使用示例      system_delay_ms(100);  // 延时100毫秒
//-------------------------------------------------------------------------------------------------------------------
void system_delay_ms (uint32 time)
{
    if(time > 40000)
    {
        while(time > 40000)
        {
            system_delay_10ns(4000000000);
            time = time - 40000;
        }
        system_delay_10ns(time * 100000);
    }
    else
    {
        system_delay_10ns(time * 100000);
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      system 延迟初始化函数
//  返回参数      void
//  使用示例      system_delay_init();
//-------------------------------------------------------------------------------------------------------------------
void system_delay_init(void)
{
    IfxStm_CompareConfig stmConfig;
    boolean interrupt_state = disableInterrupts();

    stmConfig.comparator          = IfxStm_Comparator_0;
    stmConfig.compareOffset       = IfxStm_ComparatorOffset_0;
    stmConfig.compareSize         = IfxStm_ComparatorSize_32Bits;
    stmConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmConfig.ticks               = 1;
    stmConfig.triggerPriority     = IFX_INTPRIO_STM0_SR0;
    stmConfig.typeOfService       = IfxSrc_Tos_cpu0;

    IfxStm_initCompare(&MODULE_STM0, &stmConfig);
    IfxStm_clearCompareFlag(&MODULE_STM0, IfxStm_Comparator_0);

    stmConfig.comparator          = IfxStm_Comparator_0;
    stmConfig.compareOffset       = IfxStm_ComparatorOffset_0;
    stmConfig.compareSize         = IfxStm_ComparatorSize_32Bits;
    stmConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmConfig.ticks               = 1;
    stmConfig.triggerPriority     = IFX_INTPRIO_STM1_SR0;
    stmConfig.typeOfService       = IfxSrc_Tos_cpu1;

    IfxStm_initCompare(&MODULE_STM1, &stmConfig);
    IfxStm_clearCompareFlag(&MODULE_STM1, IfxStm_Comparator_0);

    stmConfig.comparator          = IfxStm_Comparator_0;
    stmConfig.compareOffset       = IfxStm_ComparatorOffset_0;
    stmConfig.compareSize         = IfxStm_ComparatorSize_32Bits;
    stmConfig.comparatorInterrupt = IfxStm_ComparatorInterrupt_ir0;
    stmConfig.ticks               = 1;
    stmConfig.triggerPriority     = IFX_INTPRIO_STM2_SR0;
    stmConfig.typeOfService       = IfxSrc_Tos_cpu2;

    IfxStm_initCompare(&MODULE_STM2, &stmConfig);
    IfxStm_clearCompareFlag(&MODULE_STM2, IfxStm_Comparator_0);

    restoreInterrupts(interrupt_state);
}
