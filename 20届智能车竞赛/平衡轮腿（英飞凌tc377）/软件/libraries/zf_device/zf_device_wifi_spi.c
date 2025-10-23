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
* 文件名称          zf_device_wifi_spi
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.20
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
* 
* 修改记录
* 日期              作者                备注
* 2024-01-18        SeekFree            first version
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                   ------------------------------------
*                   模块管脚            单片机管脚
*                   RST                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_RST_PIN 宏定义
*                   INT                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_INT_PIN 宏定义
*                   CS                  查看 zf_device_wifi_spi.h 中 WIFI_SPI_CS_PIN 宏定义
*                   MISO                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MISO_PIN 宏定义
*                   SCK                 查看 zf_device_wifi_spi.h 中 WIFI_SPI_SCK_PIN 宏定义
*                   MOSI                查看 zf_device_wifi_spi.h 中 WIFI_SPI_MOSI_PIN 宏定义
*                   5V                  5V 电源
*                   GND                 电源地
*                   其余引脚悬空
*                   ------------------------------------
*********************************************************************************************************************/

#include "stdio.h"
#include "zf_common_clock.h"
#include "zf_common_debug.h"
#include "zf_common_fifo.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"
#include "zf_driver_spi.h"
#include "zf_device_type.h"

#include "zf_device_wifi_spi.h"

#define WIFI_CONNECT_TIME_OUT       10000       // 单位毫秒
#define SOCKET_CONNECT_TIME_OUT     50000       // 单位毫秒
#define OTHER_TIME_OUT              1000        // 单位毫秒

char wifi_spi_version[12];                      // 保存模块固件版本信息
char wifi_spi_mac_addr[20];                     // 保存模块MAC地址信息
char wifi_spi_ip_addr_port[25];                 // 保存模块IP地址与端口信息

static fifo_struct  wifi_spi_fifo;
static uint8        wifi_spi_buffer[WIFI_SPI_RECVIVE_FIFO_SIZE];
static volatile     wifi_spi_state_enum wifi_spi_mutex;
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     等待WIFI SPI就绪
// 参数说明     wait_time       最大等待时间 单位毫秒
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_wait_idle (uint32 wait_time)
{
    uint32 time = 0;
    
    wait_time = wait_time*100;
    while(0 == gpio_get_level(WIFI_SPI_INT_PIN))
    {
        system_delay_us(10);
        time++;
        if(wait_time <= time)
        {
            break;
        }
    }
    return (wait_time <= time);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入数据到WIFI SPI
// 参数说明     *buffer1        第一组需要发送的数据缓冲区地址
// 参数说明     length1         第一组数据长度
// 参数说明     *buffer2        第二组需要发送的数据缓冲区地址
// 参数说明     length2         第二组数据长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_write (const uint8 *buffer1, uint16 length1, const uint8 *buffer2, uint16 length2)
{
    gpio_low(WIFI_SPI_CS_PIN);
    if(NULL != buffer1)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer1, length1);
    }
    if(NULL != buffer2)
    {
        spi_write_8bit_array(WIFI_SPI_INDEX, buffer2, length2);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行（命令收发）
// 参数说明     *packets        发送与接收的地址
// 参数说明     length          需要接收的长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_transfer_command (wifi_spi_packets_struct *packets, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(packets->head), (uint8 *)&(packets->head), sizeof(wifi_spi_head_struct));
    
    if(length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, (const uint8 *)(packets->buffer), packets->buffer, length);
    }
    
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 发送与接收同时进行(数据收发)
// 参数说明     *write_data     发送的数据缓冲区地址
// 参数说明     *read_data      接收到的数据的存储地址
// 参数说明     length          需要接收的长度
// 返回参数     void
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static void wifi_spi_transfer_data (const uint8 *write_data, wifi_spi_packets_struct *read_data, uint16 length)
{
    gpio_low(WIFI_SPI_CS_PIN);
    
    read_data->head.command = WIFI_SPI_DATA;
    read_data->head.length  = length;
    
    spi_transfer_8bit(WIFI_SPI_INDEX, (uint8 *)&(read_data->head), (uint8 *)&(read_data->head), sizeof(wifi_spi_head_struct));
    
    if(WIFI_SPI_RECVIVE_SIZE < length)
    {
        spi_transfer_8bit(WIFI_SPI_INDEX, write_data, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
        spi_write_8bit_array(WIFI_SPI_INDEX, &write_data[WIFI_SPI_RECVIVE_SIZE], length - WIFI_SPI_RECVIVE_SIZE);
    }
    else
    {
        // 将需要发送的数据拷贝到读取缓冲区，避免出现write_data越界访问
        memcpy(read_data->buffer, write_data, length);
        spi_transfer_8bit(WIFI_SPI_INDEX, read_data->buffer, read_data->buffer, WIFI_SPI_RECVIVE_SIZE);
    }
    gpio_high(WIFI_SPI_CS_PIN);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 参数设置
// 参数说明     command         命令类型
// 参数说明     *buffer         参数地址
// 参数说明     length          参数长度
// 参数说明     wait_time       最大等待时间 单位100微妙
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_set_parameter (wifi_spi_packets_command_enum command, uint8 *buffer, uint16 length, uint32 wait_time)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = command;
        head.length  = length;
        
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }

        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), buffer, length);
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        // 接收应答信号

        head.command = WIFI_SPI_DATA;
        head.length = 0;
        wifi_spi_transfer_command((wifi_spi_packets_struct *)&head, head.length);
        system_delay_us(20);
        if(WIFI_SPI_REPLY_OK == head.command)
        {
            return_state = 0;
        }
    }while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 模块信息获取
// 参数说明     command         命令类型
// 参数说明     *buffer         保存接收到的参数地址
// 参数说明     wait_time       最大等待时间 单位100微妙
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     内部使用，用户无需关心
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_parameter (wifi_spi_packets_command_enum command, wifi_spi_packets_struct *read_data, uint32 wait_time)
{
    uint8 return_state;

    return_state = 1;
    do
    {
        // 等待从机准备就绪
        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = command;
        wifi_spi_write(&(read_data->head.command), WIFI_SPI_RECVIVE_SIZE, NULL, 0);

        if(wifi_spi_wait_idle(wait_time))
        {
            break;
        }
        read_data->head.command = WIFI_SPI_DATA;
        read_data->head.length = 0;
        wifi_spi_transfer_command(read_data, WIFI_SPI_RECVIVE_SIZE);
        return_state = 0;
    }while(0);
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 固件版本获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，固件版本信息以字符串形式保存在wifi_spi_version数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_version (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_VERSION, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_VERSION == temp_packets.head.command))
    {
        memcpy(wifi_spi_version, temp_packets.buffer, temp_packets.head.length);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI MAC地址获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_mac_addr (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_MAC_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_MAC_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_mac_addr, temp_packets.buffer, temp_packets.head.length);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI IP地址与端口号获取
// 参数说明     void            端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     调用函数之后，IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
//              需要在连接Socket之后调用此函数才能正常获取信息
//-------------------------------------------------------------------------------------------------------------------
static uint8 wifi_spi_get_ip_addr_port (void)
{
    uint8 return_state;
    wifi_spi_packets_struct temp_packets;

    return_state = wifi_spi_get_parameter(WIFI_SPI_GET_IP_ADDR, &temp_packets, OTHER_TIME_OUT);
    if((0 == return_state) && (WIFI_SPI_REPLY_IP_ADDR == temp_packets.head.command))
    {
        memcpy(wifi_spi_ip_addr_port, temp_packets.buffer, temp_packets.head.length);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的WiFi信息并尝试连接WiFi
// 参数说明     *wifi_ssid      WIFI名称
// 参数说明     *pass_word      WIFI密码
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_wifi_connect("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_wifi_connect("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_wifi_connect (char *wifi_ssid, char *pass_word)
{
    uint8 return_state;
    uint8 temp_buffer[64];
    uint16 length;
    
    if(NULL != pass_word)
    {
        // WIFI热点有密码发送热点名称与密码
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n", wifi_ssid, pass_word);
    }
    else
    {
        // WIFI热点没有密码只需要发送热点名称
        length = (uint16)sprintf((char *)temp_buffer, "%s\r\n", wifi_ssid);
    }

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_WIFI_INFORMATION, temp_buffer, length, WIFI_CONNECT_TIME_OUT);

    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 设置连接的Socket信息并尝试连接Socket
// 参数说明     *transport_type 传输类型
// 参数说明     *ip_addr        IP地址
// 参数说明     *port           目标端口号
// 参数说明     *local_port     本机端口号
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_connect("TCP", "192.168.2.5", "8080", "6060");
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_connect (char *transport_type, char *ip_addr, char *port, char *local_port)
{
    uint8 return_state;
    uint8 temp_buffer[41];
    uint16 length;
    
    length = (uint16)sprintf((char *)temp_buffer, "%s\r\n%s\r\n%s\r\n%s\r\n", transport_type, ip_addr, port, local_port);

    return_state = wifi_spi_set_parameter(WIFI_SPI_SET_SOCKET_INFORMATION, temp_buffer, length, SOCKET_CONNECT_TIME_OUT);

    // 本机IP地址与端口号信息以字符串形式保存在wifi_spi_ip_addr_port数组中
    wifi_spi_get_ip_addr_port();

    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 断开Socket连接
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例     wifi_spi_socket_disconnect();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_socket_disconnect (void)
{
    wifi_spi_packets_struct temp_packets;

    return wifi_spi_get_parameter(WIFI_SPI_CLOSE_SOCKET, &temp_packets, OTHER_TIME_OUT);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 软复位
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_reset (void)
{
    uint8 return_state;
    wifi_spi_head_struct head;
    return_state = 1;
    do
    {
        head.command = WIFI_SPI_RESET;
        head.length  = 0xA5A5;
        return_state = wifi_spi_wait_idle(OTHER_TIME_OUT);
        if(return_state)
        {
            break;
        }
        wifi_spi_write(&head.command, sizeof(wifi_spi_head_struct), NULL, 0);
    }while(0);
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI UDP模式时立即发送函数
// 参数说明     void
// 返回参数     uint8           状态 0-成功 1-错误
// 使用示例
// 备注信息     在UDP模式下模块收到数据后会等待2毫秒，2毫秒后未收到数据则将数据通过socket发送到网络，如果希望立即发送则在数据传输完毕后调用此函数
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_udp_send_now (void)
{
    uint8 return_state = 1;
    wifi_spi_packets_struct temp_packets;
    
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }

            // 立即开始socket发送
            temp_packets.head.command = WIFI_SPI_UDP_SEND;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            // 等待应答信号
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 接收应答信号
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length = 0;
            wifi_spi_transfer_command(&temp_packets, temp_packets.head.length);
            
            if(WIFI_SPI_REPLY_OK == temp_packets.head.command)
            {
                return_state = 0;
            }
            
        }while(0);
        
        // 将通讯状态设置为空闲
        wifi_spi_mutex = WIFI_SPI_IDLE;
    } 
    
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 数据块发送函数并同步接收数据
// 参数说明     *buff           需要发送的数据地址
// 参数说明     length          发送长度
// 返回参数     uint32          剩余未发送的长度
// 使用示例     wifi_spi_send_buffer(buffer, 100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 wifi_spi_send_buffer (const uint8 *buffer, uint32 length)
{
    uint16 send_length;
    wifi_spi_packets_struct temp_packets;
    
    // 检查WIFI SPI状态，如果在其他中断或者线程中已经发起了通讯，则本次不能发送数据
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        while(length)
        {
            send_length = length > WIFI_SPI_TRANSFER_SIZE ? (uint16)WIFI_SPI_TRANSFER_SIZE : (uint16)length;
            
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            wifi_spi_transfer_data(buffer, &temp_packets, send_length);
            
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
            
            length -= send_length;
            buffer += send_length;
        }
        
        // 检查最后一次的接收是否将所有的数据都接收完毕
        while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command)
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            
            // 继续读取模块剩余数据
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
    return length;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WIFI SPI 读取缓冲区
// 参数说明     *buff           接收缓冲区
// 参数说明     length          读取数据长度
// 返回参数     uint32          实际读取数据长度
// 使用示例     wifi_spi_read_buffer(buffer, 100);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint32 wifi_spi_read_buffer (uint8 *buffer, uint32 length)
{
    zf_assert(NULL != buffer);
    uint32 data_len = length;
    
#if(1 == WIFI_SPI_READ_TRANSFER)
    
    wifi_spi_packets_struct temp_packets;
    // 检查WIFI SPI状态，如果在其他中断或者线程中已经发起了通讯，则本次不能发送数据
    if(WIFI_SPI_IDLE == wifi_spi_mutex)
    {
        // 将通讯状态设置为忙
        wifi_spi_mutex = WIFI_SPI_BUSY;
        
        // 发起通讯查看模块内是否有数据未读取
        do
        {
            if(wifi_spi_wait_idle(OTHER_TIME_OUT))
            {
                break;
            }
            temp_packets.head.command = WIFI_SPI_DATA;
            temp_packets.head.length  = 0;
            wifi_spi_transfer_command(&temp_packets, WIFI_SPI_RECVIVE_SIZE);
            // 检查收到的包中是否有数据
            if((WIFI_SPI_REPLY_DATA_START == temp_packets.head.command) || (WIFI_SPI_REPLY_DATA_END == temp_packets.head.command))
            {
                // 保存接收到的数据
                if(temp_packets.head.length)
                {
                    fifo_write_buffer(&wifi_spi_fifo, temp_packets.buffer, temp_packets.head.length);
                }
            }
        }while(WIFI_SPI_REPLY_DATA_START == temp_packets.head.command);
        wifi_spi_mutex = WIFI_SPI_IDLE;
    }
#endif 
    
    fifo_read_buffer(&wifi_spi_fifo, buffer, &data_len, FIFO_READ_AND_CLEAN);
    return data_len;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     WiFi 模块初始化
// 参数说明     *wifi_ssid      目标连接的 WiFi 的名称 字符串形式
// 参数说明     *pass_word      目标连接的 WiFi 的密码 字符串形式
// 返回参数     uint8           模块初始化状态 0-成功 1-错误
// 使用示例     wifi_spi_init("SEEKFREE", "SEEKFREE123");
// 备注信息     wifi_spi_init("SEEKFREE", NULL); // 连接没有密码的WIFI热点
//-------------------------------------------------------------------------------------------------------------------
uint8 wifi_spi_init (char *wifi_ssid, char *pass_word)
{
    uint8 return_state = 0;
    
    fifo_init(&wifi_spi_fifo, FIFO_DATA_8BIT, wifi_spi_buffer, WIFI_SPI_RECVIVE_FIFO_SIZE);
    spi_init(WIFI_SPI_INDEX, SPI_MODE0, WIFI_SPI_SPEED, WIFI_SPI_SCK_PIN, WIFI_SPI_MOSI_PIN, WIFI_SPI_MISO_PIN, SPI_CS_NULL);//硬件SPI初始化
    gpio_init(WIFI_SPI_CS_PIN,  GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_RST_PIN, GPO, 1, GPO_PUSH_PULL);
    gpio_init(WIFI_SPI_INT_PIN, GPI, 0, GPI_PULL_DOWN);
    
    // 复位
    gpio_set_level(WIFI_SPI_RST_PIN, 0);
    system_delay_ms(10);
    gpio_set_level(WIFI_SPI_RST_PIN, 1);
    
    // 等待模块初始化
    system_delay_ms(100);
    wifi_spi_mutex = WIFI_SPI_IDLE;

    do
    {
        // 固件版本信息以字符串形式保存在wifi_spi_version数组中
        return_state = wifi_spi_get_version();
        if(return_state)
        {
            break;
        }

        // MAC地址信息以字符串形式保存在wifi_spi_mac_addr数组中
        wifi_spi_get_mac_addr();


        return_state = wifi_spi_wifi_connect(wifi_ssid, pass_word);
        if(return_state)
        {
            break;
        }
        
    #if(1 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
        
    #if(2 == WIFI_SPI_AUTO_CONNECT)
        return_state = wifi_spi_socket_connect("UDP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT);
        if(return_state)
        {
            break;
        }
    #endif
    }while(0);

    return return_state;
}
