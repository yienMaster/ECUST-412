/*********************************************************************************************************************
* TC264 Opensourec Library ����TC264 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC264 ��Դ���һ����
*
* TC264 ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_lora3a22
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.4
* ����ƽ̨          TC264D
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-03-29       JKS            first version
********************************************************************************************************************/


#ifndef CODE_ZF_DEVICE_LORA3A22_H_
#define CODE_ZF_DEVICE_LORA3A22_H_

#include "zf_common_headfile.h"

#define LORA3A22_UART_INDEX            (UART_2)              // ���崮��ң����ʹ�õĴ���
#define LORA3A22_UART_TX_PIN           (UART2_TX_P10_5)      // ң�������ջ���RX���� ���ӵ�Ƭ����TX����
#define LORA3A22_UART_RX_PIN           (UART2_RX_P10_6)      // ң�������ջ���TX���� ���ӵ�Ƭ����RX����
#define LORA3A22_UART_BAUDRATE         (115200)              // ָ�� lora3a22 ������ʹ�õĵĴ��ڲ�����

#define LORA3A22_DATA_LEN              ( 12  )               // lora3a22֡��
#define LORA3A22_FRAME_STAR            ( 0XA3 )              // ֡ͷ��Ϣ


typedef struct
{
    uint8 head;                                             // ֡ͷ
    uint8 sum_check;                                        // ��У��
    uint8 key[2];                                           // ҡ�˰���    ���:key[0]   �ұ�:key[1]   ����0 �ɿ�1

    int16 joystick[4];                                      //joystick[0]:���ҡ������ֵ      joystick[1]:���ҡ������ֵ
                                                            //joystick[2]:�ұ�ҡ������ֵ      joystick[3]:�ұ�ҡ������ֵ
}lora3a22_uart_transfer_dat_struct ;

extern lora3a22_uart_transfer_dat_struct lora3a22_uart_transfer;
extern uint8   lora3a22_uart_data[LORA3A22_DATA_LEN];       // lora3a22����ԭʼ����
extern vuint8  lora3a22_finsh_flag;
extern vuint8  lora3a22_state_flag;                         // ң����״̬(1��ʾ�����������ʾʧ��)
extern uint16  lora3a22_response_time;

void lora3a22_init(void);

#endif
