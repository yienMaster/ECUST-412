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
* 文件名称          zf_device_imu660ra
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
* 2023-09-15       pudding            转换实际值详细说明
* 2024-01-30       pudding            更正宏转换函数 变量增加括号
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   // 硬件 SPI 引脚
*                   SCL/SPC           查看 zf_device_imu660ra.h 中 IMU660RA_SPC_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660ra.h 中 IMU660RA_SDI_PIN 宏定义
*                   SA0/SDO           查看 zf_device_imu660ra.h 中 IMU660RA_SDO_PIN 宏定义
*                   CS                查看 zf_device_imu660ra.h 中 IMU660RA_CS_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*
*                   // 软件 IIC 引脚
*                   SCL/SPC           查看 zf_device_imu660ra.h 中 IMU660RA_SCL_PIN 宏定义
*                   SDA/DSI           查看 zf_device_imu660ra.h 中 IMU660RA_SDA_PIN 宏定义
*                   VCC               3.3V电源
*                   GND               电源地
*                   其余引脚悬空
*                   ------------------------------------
********************************************************************************************************************/

#ifndef _zf_device_imu660ra_h_
#define _zf_device_imu660ra_h_

#include "zf_common_typedef.h"

//================================================定义 IMU660RA 基本配置================================================
// IMU660RA_USE_SOFT_IIC定义为0表示使用硬件SPI驱动 定义为1表示使用软件IIC驱动
// 当更改IMU660RA_USE_SOFT_IIC定义后，需要先编译并下载程序，单片机与模块需要断电重启才能正常通讯
#define IMU660RA_USE_SOFT_IIC         (0)                                       // 默认使用硬件 SPI 方式驱动
#if IMU660RA_USE_SOFT_IIC                                                       // 这两段 颜色正常的才是正确的 颜色灰的就是没有用的
//====================================================软件 IIC 驱动====================================================
#define IMU660RA_SOFT_IIC_DELAY       (59)                                      // 软件 IIC 的时钟延时周期 数值越小 IIC 通信速率越快
#define IMU660RA_SCL_PIN              (P20_11)                                  // 软件 IIC SCL 引脚 连接 IMU660RA 的 SCL 引脚
#define IMU660RA_SDA_PIN              (P20_14)                                  // 软件 IIC SDA 引脚 连接 IMU660RA 的 SDA 引脚
//====================================================软件 IIC 驱动====================================================
#else
//====================================================硬件 SPI 驱动====================================================
#define IMU660RA_SPI_SPEED            (10 * 1000 * 1000)                        // 硬件 SPI 速率
#define IMU660RA_SPI                  (SPI_0)                                   // 硬件 SPI 号
#define IMU660RA_SPC_PIN              (SPI0_SCLK_P20_11)                        // 硬件 SPI SCK 引脚
#define IMU660RA_SDI_PIN              (SPI0_MOSI_P20_14)                        // 硬件 SPI MOSI 引脚
#define IMU660RA_SDO_PIN              (SPI0_MISO_P20_12)                        // 硬件 SPI MISO 引脚
//====================================================硬件 SPI 驱动====================================================
#endif
#define IMU660RA_CS_PIN               (P20_13)                                  // CS 片选引脚
#define IMU660RA_CS(x)                ((x) ? (gpio_high(IMU660RA_CS_PIN)) : (gpio_low(IMU660RA_CS_PIN)))
typedef enum
{
    IMU660RA_ACC_SAMPLE_SGN_2G ,                                                // 加速度计量程 ±2G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    IMU660RA_ACC_SAMPLE_SGN_4G ,                                                // 加速度计量程 ±4G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    IMU660RA_ACC_SAMPLE_SGN_8G ,                                                // 加速度计量程 ±8G  (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
    IMU660RA_ACC_SAMPLE_SGN_16G,                                                // 加速度计量程 ±16G (ACC = Accelerometer 加速度计) (SGN = signum 带符号数 表示正负范围) (G = g 重力加速度 g≈9.80 m/s^2)
}imu660ra_acc_sample_config;

typedef enum
{
    IMU660RA_GYRO_SAMPLE_SGN_125DPS ,                                           // 陀螺仪量程 ±125DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    IMU660RA_GYRO_SAMPLE_SGN_250DPS ,                                           // 陀螺仪量程 ±250DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    IMU660RA_GYRO_SAMPLE_SGN_500DPS ,                                           // 陀螺仪量程 ±500DPS  (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    IMU660RA_GYRO_SAMPLE_SGN_1000DPS,                                           // 陀螺仪量程 ±1000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
    IMU660RA_GYRO_SAMPLE_SGN_2000DPS,                                           // 陀螺仪量程 ±2000DPS (GYRO = Gyroscope 陀螺仪) (SGN = signum 带符号数 表示正负范围) (DPS = Degree Per Second 角速度单位 °/S)
}imu660ra_gyro_sample_config;

#define IMU660RA_ACC_SAMPLE_DEFAULT   ( IMU660RA_ACC_SAMPLE_SGN_8G )            // 在这设置默认的 加速度计 初始化量程
#define IMU660RA_GYRO_SAMPLE_DEFAULT  ( IMU660RA_GYRO_SAMPLE_SGN_2000DPS )      // 在这设置默认的 陀螺仪   初始化量程
#define IMU660RA_TIMEOUT_COUNT        (0x00FF)                                  // IMU660 超时计数
//================================================定义 IMU660RA 基本配置================================================


//================================================定义 IMU660RA 内部地址================================================
#define IMU660RA_DEV_ADDR             (0x69)                                    // SA0接地：0x68 SA0上拉：0x69 模块默认上拉
#define IMU660RA_SPI_W                (0x00)
#define IMU660RA_SPI_R                (0x80)

#define IMU660RA_CHIP_ID              (0x00)
#define IMU660RA_PWR_CONF             (0x7C)
#define IMU660RA_PWR_CTRL             (0x7D)
#define IMU660RA_INIT_CTRL            (0x59)
#define IMU660RA_INIT_DATA            (0x5E)
#define IMU660RA_INT_STA              (0x21)
#define IMU660RA_ACC_ADDRESS          (0x0C)
#define IMU660RA_GYRO_ADDRESS         (0x12)
#define IMU660RA_ACC_CONF             (0x40)
#define IMU660RA_ACC_RANGE            (0x41)
#define IMU660RA_GYR_CONF             (0x42)
#define IMU660RA_GYR_RANGE            (0x43)

#define IMU660RA_ACC_SAMPLE           (0x02)                                    // 加速度计量程
//                                以下为25℃的转换比例 IMU660RA的加速度温漂系数为 0.004%/K  常值零偏的温度敏感系数为 ±0.25mg/K
// 设置为:0x00 加速度计量程为:±2g         获取到的加速度计数据 除以 16384   可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x01 加速度计量程为:±4g         获取到的加速度计数据 除以 8192    可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x02 加速度计量程为:±8g         获取到的加速度计数据 除以 4096    可以转化为带物理单位的数据 单位：g(m/s^2)
// 设置为:0x03 加速度计量程为:±16g        获取到的加速度计数据 除以 2048    可以转化为带物理单位的数据 单位：g(m/s^2)

#define IMU660RA_GYR_SAMPLE           (0x00)                                    // 陀螺仪量程
//                                以下为25℃的转换比例 IMU660RA的陀螺仪温漂系数为 0.02%/K  常值零偏的温度敏感系数为 ±0.015dps/K
// 设置为:0x00 陀螺仪量程为:±2000dps      获取到的陀螺仪数据除以16.384          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x01 陀螺仪量程为:±1000dps      获取到的陀螺仪数据除以32.768          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x02 陀螺仪量程为:±500 dps      获取到的陀螺仪数据除以65.536          可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x03 陀螺仪量程为:±250 dps      获取到的陀螺仪数据除以131.072         可以转化为带物理单位的数据，单位为：°/s
// 设置为:0x04 陀螺仪量程为:±250 dps      获取到的陀螺仪数据除以262.144         可以转化为带物理单位的数据，单位为：°/s
//================================================定义 IMU660RA 内部地址================================================


//================================================声明 IMU660RA 全局变量================================================
extern int16 imu660ra_gyro_x, imu660ra_gyro_y, imu660ra_gyro_z;                 // 三轴陀螺仪数据      GYRO (陀螺仪)
extern int16 imu660ra_acc_x, imu660ra_acc_y, imu660ra_acc_z;                    // 三轴加速度计数据     ACC  (accelerometer 加速度计)
extern float imu660ra_transition_factor[2];                                     // 转换实际值的比例
//================================================声明 IMU660RA 全局变量================================================


//================================================声明 IMU660RA 基础函数================================================
void  imu660ra_get_acc              (void);                                     // 获取 IMU660RA 加速度计数据
void  imu660ra_get_gyro             (void);                                     // 获取 IMU660RA 陀螺仪数据
uint8 imu660ra_init                 (void);                                     // 初始化 IMU660RA
//================================================声明 IMU660RA 基础函数================================================


//================================================声明 IMU660RA 拓展函数================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 加速度计数据转换为实际物理数据
// 参数说明     acc_value       任意轴的加速度计数据
// 返回参数     void
// 使用示例     float data = imu660ra_acc_transition(imu660ra_acc_x);           // 单位为 g(m/s^2)
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define imu660ra_acc_transition(acc_value)      ((float)(acc_value) / imu660ra_transition_factor[0])

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将 IMU660RA 陀螺仪数据转换为实际物理数据
// 参数说明     gyro_value      任意轴的陀螺仪数据
// 返回参数     void
// 使用示例     float data = imu660ra_gyro_transition(imu660ra_gyro_x);         // 单位为 °/s
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
#define imu660ra_gyro_transition(gyro_value)    ((float)(gyro_value) / imu660ra_transition_factor[1])
//================================================声明 IMU660RA 拓展函数================================================

#endif

