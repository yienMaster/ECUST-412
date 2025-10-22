/*********************************************************************************************************************
* TC364 Opensourec Library ����TC364 ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� TC364 ��Դ���һ����
*
* TC364 ��Դ�� ��������
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
* �ļ�����          cpu0_main
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          ADS v1.9.20
* ����ƽ̨          TC364DP
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2022-11-02       pudding            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������

// **************************** �������� ****************************

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������

    system_delay_ms(500);
    CPU0_Init();

    // �˴���д�û����� ���������ʼ�������
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

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

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
// **************************** �������� ****************************
