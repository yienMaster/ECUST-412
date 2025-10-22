#ifndef __UPPER_COMPUTER_H__
#define __UPPER_COMPUTER_H__

#include "zf_common_headfile.h"

//PID������ʾ�ṹ��
typedef struct
{
        float p;
        float i;
        float d;
        uint8 name[20];
}PID_Display_Structure;


typedef struct
{
        uint8 row[16];
        uint8 col[16];
}Point_Display_Structure;


//WIFI����
#define SSID            "Xiaomi 14"
#define PASSWORD        "zkz167294381"
#define TARGET_IP       "192.168.128.38"

//#define SSID            "cd"
//#define PASSWORD        "25802580"
//#define TARGET_IP       "192.168.75.27"

#define TARGET_PORT     "8888"
#define SOURCE_PORT     "6666"

//LCD��ʾ����
#define LCD_PARA_START_X        (0)
#define LCD_PARA_START_Y        (45)
#define LCD_OFFSET_X            (50)
#define LCD_OFFSET_Y            (8)

//�Ƿ�ͼ��
#define UPPER_COMPUTER_IMAGE    (0)
#define LCD_IMAGE               (1)


extern uint8 image_display[MT9V03X_H][MT9V03X_W];
extern Point_Display_Structure point_u_display,point_d_display,point_m_display;
extern PID_Display_Structure motor_l_display,motor_r_display,dir_display,target_speed_display;
extern uint8 image_send_flag;
extern uint8 para_update_flag;
extern uint8 para_update_display_flag;
extern float send_buffer_1[100],send_buffer_2[100];
extern uint16 send_buffer_index;


void WIFI_Init(void);                   //WIFI��ʼ��
void Connection_Init(void);             //��λ��TCP���ӳ�ʼ��
void ZF_Assistance_Init(void);          //������ֳ�ʼ��
void Msg_Process(uint8* rxbuf);         //�����ַ���������
void Receive_msg(void);                 //�����ַ���
void PID_display_init(void);            //PID��ʾ������ʼ��
void LCD_display(void);                 //LCD��ʾ����
void Copy_Image_And_Point(void);         //��ȡ��ʾ��ͼ��ͱ���

#endif
