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
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC364DP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-02       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

// **************************** 代码区域 ****************************

int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等

    system_delay_ms(500);
    CPU0_Init();

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕
    while (TRUE)
    {
        // 此处编写需要循环执行的代码

        if(image_send_flag)
        {
            image_send_flag=0;
#if USE_UPPER_COMPUTER
#if UPPER_COMPUTER_IMAGE
            seekfree_assistant_camera_send();
#endif
            seekfree_assistant_oscilloscope_data.data[0]=speed_L;
            seekfree_assistant_oscilloscope_data.data[1]=speed_R;
            seekfree_assistant_oscilloscope_data.data[2]=target_speed_L;
            seekfree_assistant_oscilloscope_data.data[3]=target_speed_R;
            seekfree_assistant_oscilloscope_data.data[4]=motor_l.out;
            seekfree_assistant_oscilloscope_data.data[5]=motor_r.out;
//            seekfree_assistant_oscilloscope_data.data[5]=point_distance;
//
//
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
#endif


#if USE_LCD
//            LCD_display();
//            tft180_show_float(80,40,target_speed_set,3,1);
//
//            tft180_show_uint(100,0,point_distance,2);
//            tft180_show_uint(100,10,start_flag,1);
//            tft180_show_uint(100,20,run_state,2);
            if(!start_flag)
            {
                Menu_Display();
//                tft180_show_float(90,0,target_speed_pid.out,2,1);
            }


#endif
        }

#if USE_UPPER_COMPUTER
        Receive_msg();
#endif

        // 此处编写需要循环执行的代码
    }
}

#pragma section all restore
// **************************** 代码区域 ****************************
