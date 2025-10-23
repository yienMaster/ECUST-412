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
* 文件名称          cpu1_main
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
#pragma section all "cpu1_dsram"
#include "image.h"
#include "adc.h"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU0的RAM中


// *************************** 例程硬件连接说明 ***************************
// 使用逐飞科技 英飞凌 调试下载器连接
//      直接将下载器正确连接在核心板的调试下载接口即可
//
// 接入 高速Wifi SPI 模块
//      模块管脚            单片机管脚
//      RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
//      INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
//      CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
//      MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
//      SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
//      MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
//      5V                  5V 电源
//      GND                 电源地
// 接入总钻风灰度数字摄像头 对应主板摄像头接口 请注意线序
//      模块管脚            单片机管脚
//      TXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_TX 宏定义
//      RXD                 查看 zf_device_mt9v03x.h 中 MT9V03X_COF_UART_RX 宏定义
//      PCLK                查看 zf_device_mt9v03x.h 中 MT9V03X_PCLK_PIN 宏定义
//      VSY                 查看 zf_device_mt9v03x.h 中 MT9V03X_VSYNC_PIN 宏定义
//      D0-D7               查看 zf_device_mt9v03x.h 中 MT9V03X_DATA_PIN 宏定义 从该定义开始的连续八个引脚
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源
// 接入1.8寸TFT模块
//      模块管脚            单片机管脚
//      SCL                 查看 zf_device_tft180.h 中 TFT180_SCL_PIN 宏定义 默认 P15_3
//      SDA                 查看 zf_device_tft180.h 中 TFT180_SDA_PIN 宏定义 默认 P15_5
//      RES                 查看 zf_device_tft180.h 中 TFT180_RES_PIN 宏定义 默认 P15_1
//      DC                  查看 zf_device_tft180.h 中 TFT180_DC_PIN  宏定义 默认 P15_0
//      CS                  查看 zf_device_tft180.h 中 TFT180_CS_PIN  宏定义 默认 P15_2
//      BL                  查看 zf_device_tft180.h 中 TFT180_BL_PIN  宏定义 默认 P15_4
//      GND                 核心板电源地 GND
//      3V3                 核心板 3V3 电源


// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.查看电脑所连接的wifi，记录wifi名称，密码，IP地址
//
// 3.在下方的代码区域中修改宏定义，WIFI_SSID_TEST为wifi名称，WIFI_PASSWORD_TEST为wifi密码
//
// 4.打开zf_device_wifi_spi.h，修改WIFI_SPI_TARGET_IP宏定义，设置为电脑wifi的IP地址
//
// 5.下载例程到单片机中，打开串口助手，打开下载器的串口
//
// 6.打开逐飞科技的逐飞助手软件，选择图像传输功能
//
// 7.选择网络，设置为TCP Server，本机地址中选择WIFI网络然后点击链接


// *************************** 例程测试说明 ***************************
// 1.本例程会通过 Debug 串口输出测试信息 请务必接好调试串口以便获取测试信息
//
// 2.连接好模块和核心板后（尽量使用配套主板测试以避免供电不足的问题） 烧录本例程 按下复位后程序开始运行
//
// 3.如果模块未能正常初始化 会通过 DEBUG 串口输出未能成功初始化的原因 随后程序会尝试重新初始化 一般情况下重试会成功
//
// 4.如果一直在 Debug 串口输出报错 就需要检查报错内容 并查看本文件下方的常见问题列表进行排查
//
// 5.程序默认不开启 WIFI_SPI_AUTO_CONNECT 宏定义 通过 main 函数中的接口建立网络链接 如果需要固定自行建立链接 可以开启该宏定义
//
// 6.当模块初始化完成后会通过 DEBUG 串口输出当前模块的主要信息：固件版本、IP信息、MAC信息、PORT信息
//
// 7.本例程是 TCP Client 例程 模块会被配置为 TCP Client 需要连接到局域网内的 TCP Server 才能进行通信
//   目标连接的 TCP Server 的 IP 与端口默认使用 zf_device_wifi_spi.h 中 WIFI_SPI_TARGET_IP 与 WIFI_SPI_TARGET_PORT 定义
//   实际测试需要根据现场 TCP Server 的实际 IP 地址与端口设置
//
// 8.当本机设备主动连接到 TCP Server （例如电脑使用逐飞助手上位机进入 TCP Server 模式 然后本机连接到电脑的 IP 与端口）
//   本例程会采集总钻风图像并发送到逐飞助手上位机
//
// 9.默认情况下逐飞助手显示摄像头的图像帧率可以达到50帧，如果无线网络比较复杂例如附近有较多的WIFI热点，可能会导致显示帧率较低
//
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 将核心板插在主板上 插到底
// 2.摄像头接在主板的摄像头接口 注意线序1.8寸TFT模块插入主板屏幕接口
// 3.主板上电 或者核心板链接完毕后上电 核心板按下复位按键
// 4.屏幕会显示初始化信息然后显示摄像头图像
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查


// **************************** 代码区域 ****************************

//0：不包含边界信息
//1：包含三条边线信息，边线信息只包含横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点
//2：包含三条边线信息，边界信息只含有纵轴坐标，横轴坐标由图像宽度得到，意味着每个边界在一列中只会有一个点，一般来说很少有这样的使用需求
//3：包含三条边线信息，边界信息含有横纵轴坐标，意味着你可以指定每个点的横纵坐标，边线的数量也可以大于或者小于图像的高度，通常来说边线数量大于图像的高度，一般是搜线算法能找出回弯的情况
//4：没有图像信息，仅包含三条边线信息，边线信息只包含横轴坐标，纵轴坐标由图像高度得到，意味着每个边界在一行中只会有一个点，这样的方式可以极大的降低传输的数据量
#define buzzer_pwm                  (ATOM2_CH0_P33_10)
#define LED2                        (P32_7)
//#define LED2                        (P21_4)

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    // 初始化 LED2 输出 默认高电平 推挽输出模式
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    // 推荐先初始化摄像头，后初始化逐飞助手
    mt9v03x_init();
    //初始化蜂鸣器
    pwm_init(buzzer_pwm , 2700 , 0);

    // 此处编写用户代码 例如外设初始化代码等
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
    while (TRUE)
    {
        gpio_toggle_level(LED2);
        //读取图像
        if(mt9v03x_finish_flag)
        {
            mt9v03x_finish_flag = 0;
            //图像处理
            image_process();
            //低压报警
            if((voltage_power_supply <= 10.4) && (voltage_power_supply >= 4))
            {
                pwm_set_duty(buzzer_pwm, 5000);
            }
            //特殊元素警示
            if((buzzer == 1)&&(voltage_power_supply > 4))
                pwm_set_duty(buzzer_pwm, 5000);
            else
                pwm_set_duty(buzzer_pwm, 0);

        }
        // 此处编写需要循环执行的代码
    }
}
#pragma section all restore
