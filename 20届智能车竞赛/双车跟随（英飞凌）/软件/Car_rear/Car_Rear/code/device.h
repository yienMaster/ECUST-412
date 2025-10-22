#ifndef __DEVICE_H__
#define __DEVICE_H__

#include "zf_common_headfile.h"

//��������LED������

//����ö��
typedef enum
{
    key1,key2,key3,key4,nokey
}Key_enum;

//����״̬ö��
typedef enum
{
    idle,           //δ����
    maypress,       //���ܰ���
    press,          //ȷ������
    release         //�ͷ�
}Key_State_enum;

typedef enum
{
    speed_120,
    speed_135,
    speed_150,
    speed_160,
    speed_170,
    speed_180,
    speed_190,
    speed_200,
    speed_210,
    speed_220,
    speed_230,
    speed_240,
    speed_idle
}Speed_enum;

typedef enum
{
    page_main,page_setting,page_pid_choose,page_value_set
}Menu_enum;

typedef enum
{
    menu_motor_l,menu_motor_r,menu_dir,
    menu_target_speed,menu_target_offset_coef,
    menu_target_distance_coef,menu_dynamic_p_coef,menu_bldc,menu_target_distance,
    menu_para_count
}Menu_Para_enum;

typedef enum
{
    menu_p,menu_i,menu_d,
    menu_pid_count
}Menu_PID_enum;


//���Ŷ���
#define LED_PIN             (P00_12)        //����LED
#define BUZZER_PIN          (P33_3)         //������
#define KEY1                (P11_11)        //����
#define KEY2                (P11_9)
#define KEY3                (P13_2)
#define KEY4                (P13_0)

//�豸���غ�����0��1����
#define LED(x)              (gpio_set_level(LED_PIN, (!x)))
#define BUZZER(x)           (gpio_set_level(BUZZER_PIN, (x))
#define DEVICE_TOGGLE(x)    (gpio_toggle_level(x))


extern Key_enum key_press;
extern Speed_enum speed_para_set;
extern uint8 bldc_duty;


//��ʼ������
void Led_Init(void);
void Buzzer_Init(void);
void Key_Init(void);

//�������
void Key_Scan(void);

//�ٶȶ�Ӧ��������
void Speed_Para_Update(void);

void Menu_Update(void);
void Menu_Display(void);


#endif
