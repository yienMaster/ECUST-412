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
* 文件名称          cpu0_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 tc264 V2.6主板 按照下述方式进行接线
//      模块引脚    单片机引脚
//      RX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_TX  宏定义 默认 P15_7
//      TX          查看 small_driver_uart_control.h 中 SMALL_DRIVER_RX  宏定义 默认 P15_6
//      GND         GND


// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电 连接 CYT2BL3 FOC 双驱
// 2.如果初次使用 请先点击双驱上的MODE按键 以矫正零点位置 矫正时 电机会发出音乐
// 3.可以在逐飞助手上位机上看到如下串口信息：
//      left speed:xxxx, right speed:xxxx
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************
#include "zf_common_headfile.h"
#include "small_driver_uart_control.h"
#include "tft.h"
#include "zf_device_lora3a22.h"
#pragma section all "cpu0_dsram"
#include "isr_config.h"
#include "control.h"
#include "jump_control.h"
#include"engine.h"


// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中

// *************************** 例程硬件连接说明 ***************************
// lora3a22无线串口模块
//      模块管脚            单片机管脚
//      RX                  查看 zf_device_lora3a22.h 中 LORA3A22_UART_RX_PIN  宏定义 默认 P10_6
//      TX                  查看 zf_device_lora3a22.h 中 LORA3A22_UART_TX_PIN  宏定义 默认 P10_5
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源

// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程，单独使用核心板与调试下载器或者 USB-TTL 模块，并连接好编码器，在断电情况下完成连接
// 2.将调试下载器或者 USB-TTL 模块连接电脑 完成上电
// 3.电脑上使用串口助手打开对应的串口，串口波特率为 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义 默认 115200，核心板按下复位按键
// 4.可以在串口助手上看到如下串口信息：
//      head = 163
//      sum_check = 165
//      key0 = 1
//      key1 = 1
//      joystick[0] = 0
//      joystick[1] = 0
//      joystick[2] = 0
//      joystick[3] = 0
// 5.操作摇杆数据发送变化
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

#define PIT_NUM                 (CCU61_CH1 )                            // 使用的周期中断编号
#define PIT_period              (10 )                                   // 中断周期
#define LED1                    (P20_0)
//#define LED1                    (P20_9)
int bridge_high = 0;
//0 是低单边桥
//1 是高单边桥
Attitude_3D_Kalman filter;
IMU_t IMU_data;
float angel_init =0;
int duty = 1;
int stop = 0;
float pitch1,roll1,yaw1;
float v_buchang;
/*腿部姿态设置*/
float x_current,y_current;
extern pid_param_t engine_high;   // 发动机高度PID参数
int stop_flash = 0;
int Bridge_position = 1;
int yanshi_biaozhiwei=100;
int change_speed = 0;
int core0_main(void)
{
    clock_init();                   // 获取时钟频率<务必保留>
    debug_init();                   // 初始化默认调试串口
    // 此处编写用户代码 例如外设初始化代码等
    x_current=0;
    y_current=0.04;
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL); // 初始化 LED1 输出 默认高电平 推挽输出模式

        while(1)
        {

            if(imu963ra_init())
            {
               printf("\r\nIMU963RA init error.");// IMU963RA 初始化失败
            }
            else
            {
               break;
            }
            gpio_toggle_level(LED1);// 翻转 LED 引脚输出电平 控制 LED 亮灭 初始化出错这个灯会闪的很慢
        }
    small_driver_uart_init();       // 初始化驱动通讯功能
    lora3a22_init();                //初始化遥控器
    Balance_init();
    pit_ms_init(PIT_NUM, PIT_period);// 初始化 CCU61_CH1 为周期中断 10ms 周期
    pit_ms_init(CCU60_CH0, 4);      //获取九轴信息
    pit_ms_init(CCU60_CH1, 4);      //开启直立速度方向环PID中断
    pit_ms_init(CCU61_CH0, 20);    //开启地形自适应PID中断
    jump_state = JUMP_FREE;
    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();         // 等待所有核心初始化完毕

//    system_delay_ms(1000);         //直立平稳后加速
//    target_velocity = 240;         //设置目标速度
//    target_motor_Stand = 1;
    while (TRUE)
    {
//        // 此处编写需要循环执行的代码
/****************************************自适应pid调节+模糊pid调节***************************************/
       // adjust_pid_based_on_leg_height(&y_current);
/***********************************灯**************************************************/
        gpio_toggle_level(LED1);
/**********此处执行斑马线代码（状态机)*************************************/
        if(stop_position == 1 && stop == 0)
        {
            stop = 1;
            system_delay_ms(500);
            stop_position = 0;
        }
        if(stop_position == 1 && stop == 1)
        {
            now_start = 0;
            target_velocity = 0;
            system_delay_ms(1000);
            pit_all_close ();
            small_driver_set_duty(0, 0);
            stop = 0;
            stop_flash = 1;
        }
/**********此处执行跳跃代码（状态机)****************************************/
        if((jump_position==1)&&(BridgeState != SINGLE_BRIDGE_ACTIVE))
        {

          jump_state=JUMP_PREPARE;
          jump_process_control(&x_current,&y_current);
          system_delay_ms(100);
          jump_position=0;
          //保证速度稳定后，急停
          // system_delay_ms(2000);
         }
/**********此处执行跳跃绕弯代码（状态机)****************************************/



/***********************腿部自适应(过单边桥启动)**************************/
         if ((BridgeState == SINGLE_BRIDGE_ACTIVE)||(yanshi_biaozhiwei<=40))
         {
             if(BridgeState == SINGLE_BRIDGE_ACTIVE)
             {
                 while(Bridge_position)
                 {
                     if(bridge_high == 0)//低单边桥
                     {
 /***********************斑马线前**************************/
                         now_start = 0;
                         target_velocity = 400;
                         system_delay_ms(500);
                         //target_velocity = 440;//600
                         target_velocity = 500;
                         Bridge_position = 0;//开始腿部自适应
 /***********************弯道后**************************/
//                         now_start = 0;
//                         target_velocity = 350;
//                         system_delay_ms(500);
//                         target_velocity = 400;
//                         Bridge_position = 0;//开始腿部自适应
                     }
                     else if(bridge_high == 1)//高单边桥
                     {
                         now_start = 0;
                         target_velocity = 50;
                         system_delay_ms(500);
                         y_current=0.07;
                         target_motor_Stand = 1;
                        system_delay_ms(100);
                        target_velocity = 150;
                       Bridge_position = 0;//开始腿部自适应
                     }
                 }
             }
         }
         else
         {
             if(Bridge_position == 0)
             {
                 yanshi_biaozhiwei = 100;
                 if(bridge_high == 0)//低单边桥
                 {
//                     target_velocity = 100;
                     system_delay_ms(300);
                     Bridge_position = 1;//退出腿部自适应
                     now_start=V_a;//之前的速度
                 }
                 else if(bridge_high == 1)//高单边桥
                 {
                     target_velocity = 50;
                     system_delay_ms(300);
                     y_current=0.04;
                     target_motor_Stand = 2.2;
                     system_delay_ms(500);
                     Bridge_position = 1;//退出腿部自适应
                     now_start=V_a;//之前的速度
                 }
             }
         }
/************此处是遥控器代码***************************************************/
//        if (lora3a22_state_flag == 1)
//        {
//            if (lora3a22_finsh_flag == 1)
//            {
//
//
//               lora3a22_finsh_flag = 0;
//               if(lora3a22_uart_transfer.key[1] == 0)//停止ֹ
//               {
//                   target_velocity = 0;
//
//               }
//
//
//               target_velocity = (float)lora3a22_uart_transfer.joystick[3] / 2000 * 600;
//               if(lora3a22_uart_transfer.key[0] == 0)//停止ֹ
//                       {
//                              jump_state=JUMP_PREPARE;
//                              jump_process_control(&x_current,&y_current);
//                              //保证速度稳定后，急停
//                              system_delay_ms(100);
//                      }
//            }
//
//        }
//        else
//        {
//   //         printf("lora3a22 connection fail \r\n");
//            //target_velocity = 0;
//            //engine_change = 750;
//        }
/************此处是遥控器代码***************************************************/
//        if(target_velocity==300)
//        {
//           target_velocity=0;
//        }
//        else if(target_velocity==0)
//          {
//              target_velocity=300;
//          }
//
//        system_delay_ms(2000);
        // 此处编写需要循环执行的代码
    }
}


IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)
{
    /******************中断imu的读取**********/
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH0);
    //开始中断
    imu963ra_get_acc();                            // 获取 IMU963RA 的加速度测量数值
    imu963ra_get_gyro();                           // 获取 IMU963RA 的角速度测量数值
    imu963ra_get_mag();                           // 获取 IMU963RA 的地磁计测量数值
    /********************卡尔曼滤波**********************************/
    cal(imu963ra_acc_x,  imu963ra_acc_y,  imu963ra_acc_z, imu963ra_gyro_x, imu963ra_gyro_y, imu963ra_gyro_z,
                      imu963ra_mag_x,   imu963ra_mag_y,  imu963ra_mag_z);
    Kalman_update(&IMU_data.filter_result,&filter,IMU_data.accel[0],IMU_data.accel[1],IMU_data.accel[2],IMU_data.gyro[0],IMU_data.gyro[1],IMU_data.gyro[2],IMU_data.mag[0],IMU_data.mag[1],IMU_data.mag[2]);
    /*********截断滤波****************************/
    if(func_abs(pitch1 - IMU_data.filter_result.pitch) <= 0.4)
        IMU_data.filter_result.pitch = pitch1;
    if(func_abs(roll1 - IMU_data.filter_result.roll) <= 0.1)
       IMU_data.filter_result.roll = roll1;
    if(func_abs(yaw1 - IMU_data.filter_result.yaw) <= 1)
        IMU_data.filter_result.yaw = yaw1;
   //printf("data: %f,%f,%f\r\n", IMU_data.filter_result.pitch,IMU_data.filter_result.roll,IMU_data.filter_result.yaw);
    pitch1 = IMU_data.filter_result.pitch;
    roll1 = IMU_data.filter_result.roll;
    yaw1 = IMU_data.filter_result.yaw;

}
IFX_INTERRUPT(cc60_pit_ch1_isr, CCU6_0_CH1_INT_VECTAB_NUM, CCU6_0_CH1_ISR_PRIORITY)
{
    /*********平衡姿态控制***************/
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU60_CH1);
    //中断开始
    balance_control();

 }
IFX_INTERRUPT(cc61_pit_ch0_isr, CCU6_1_CH0_INT_VECTAB_NUM, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);                     // 开启中断嵌套
    pit_clear_flag(CCU61_CH0);
    /************     速度递增     ******************/
    if(now_start > 0)
    {
        if(((target_velocity*0.8) <= now_velocity) || target_velocity == 0)
        {
            if(change_speed == 0)
            {
                target_velocity = target_velocity + 150;
            }
            else
            {
                target_velocity = target_velocity + 100;
            }
            if(target_velocity >= now_start)
            {
                target_velocity = now_start;
            }
        }
        if(now_start <= now_velocity)
        {
            now_start=0;
        }
    }
    /************     丢速度递增(坡道一定去掉)     ******************/
    if(((target_velocity*0.3) >= now_velocity) && now_start==0
            && target_velocity >= 200 && Bridge_position == 1)
    {
        now_start = V_a;
        target_velocity = now_velocity + 50;
    }
    /************     直线增速     ******************/
//    if(speed_up == 1)
//    {
//        change_speed = 1;
//    }
//    else
//    {
//        change_speed = 0;
//        now_start = V_a;
//    }
    /************腿部自适应(单边桥)******************/
    if(Bridge_position == 0)
    {
        if ((BridgeState == SINGLE_BRIDGE_ACTIVE)||(yanshi_biaozhiwei<=50))
        {
            if(BridgeState == SINGLE_BRIDGE_ACTIVE)
            {
                yanshi_biaozhiwei=0;
                leg_roll_control(y_current, IMU_data.filter_result.pitch );
            }
            else if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
            {
                yanshi_biaozhiwei++;
                leg_roll_control(y_current, IMU_data.filter_result.pitch );
            }
        }
    }
    /************腿部自适应******************/
    if(jump_state == JUMP_FREE && Bridge_position == 1)//不跳跃的时候，不单边桥时
    {
      leg_control(&x_current,&y_current);
    }


}

IFX_INTERRUPT(cc61_pit_ch1_isr, CCU6_1_CH1_INT_VECTAB_NUM, CCU6_1_CH1_ISR_PRIORITY)
{
    /************遥控器接受中断**/
    interrupt_global_enable(0);                      // 开启中断嵌套
    pit_clear_flag(CCU61_CH1);
//    //中断开始
//    lora3a22_response_time++;
//
//    if (lora3a22_response_time > 500 / PIT_period)   //500ms 没有接受倒数据判断位发送端异常
//    {
//        lora3a22_state_flag = 0;                     //遥控器状态位清零
//        lora3a22_response_time = 0;
//    }
}
void dl1a_int_handler (void)
{
#if DL1A_INT_ENABLE
    dl1a_get_distance();
#endif

}
#pragma section all restore
// **************************** 代码区域 ****************************
// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 问题1：串口没有数据
//      查看逐飞助手上位机打开的是否是正确的串口，检查打开的 COM 口是否对应的是调试下载器或者 USB-TTL 模块的 COM 口
//      如果是使用逐飞科技 英飞凌TriCore 调试下载器连接，那么检查下载器线是否松动，检查核心板串口跳线是否已经焊接，串口跳线查看核心板原理图即可找到
//      如果是使用 USB-TTL 模块连接，那么检查连线是否正常是否松动，模块 TX 是否连接的核心板的 RX，模块 RX 是否连接的核心板的 TX
// 问题2：串口数据乱码
//      查看逐飞助手上位机设置的波特率是否与程序设置一致，程序中 zf_common_debug.h 文件中 DEBUG_UART_BAUDRATE 宏定义为 debug uart 使用的串口波特率
// 问题3：无刷电机无反应
//      检查Rx信号引脚是否接对，信号线是否松动
// 问题4：无刷电机转动但转速显示无速度
//      检查Tx信号引脚是否接对，信号线是否松动
