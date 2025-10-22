#ifndef __UPPER_COMPUTER_H__
#define __UPPER_COMPUTER_H__

#include "zf_common_headfile.h"

//PID参数显示结构体
typedef struct
{
        float p;
        float i;
        float d;
        uint8 name[20];
}PID_Display_Structure;

typedef struct
{
        uint8 row[MT9V03X_H<<1];
        uint8 col[MT9V03X_H<<1];
}Boundary_Display_Structure;



//WIFI参数
//#define SSID            "SmartCar"
//#define PASSWORD        "ABCD996996"
//#define TARGET_IP       "192.168.9.4"

#define SSID            "Xiaomi 14"
#define PASSWORD        "zkz167294381"
#define TARGET_IP       "192.168.194.38"

#define TARGET_PORT     "8888"
#define SOURCE_PORT     "6666"

//LCD显示参数
#define LCD_PARA_START_X        (1)
#define LCD_PARA_START_Y        (45)
#define LCD_OFFSET_X            (43)
#define LCD_OFFSET_Y            (10)

//上位机显示元素状态参数
#define STATE_IDLE_DISPLAY          (0x1FFFFFF)
#define STATE_STRAIGHT_DISPLAY      (0x421084)
#define STATE_CROSSING_DISPLAY      (0x427C84)
#define STATE_ZEBRA_DISPLAY         (0x15AD6B5)
#define STATE_CURVE_L_DISPLAY       (0x421041)
#define STATE_CURVE_R_DISPLAY       (0x421110)
#define STATE_ROUND_L_DISPLAY       (0x16CE736)
#define STATE_ROUND_R_DISPLAY       (0xD9CE6D)


//是否图传
#define UPPER_COMPUTER_IMAGE    (1)
#define LCD_IMAGE               (1)


extern uint8 image_display[MT9V03X_H][MT9V03X_W];
extern uint8 binary_image_display[MT9V03X_H][MT9V03X_W];
extern PID_Display_Structure motor_l_display,motor_r_display,dir_display;
extern Boundary_Display_Structure boundary_l_display,boundary_r_display,boundary_m_display;
extern float kp2_display,gkd_display;
extern uint8 image_send_flag;
extern uint8 para_update_flag;
extern uint8 para_update_display_flag;
extern uint8 point_index,point_index_display;
extern float send_buffer_1[100],send_buffer_2[100];
extern uint16 send_buffer_index;
extern float R2_left_display,R2_right_display;
extern uint8 test_uint8_display;


void WIFI_Init(void);                   //WIFI初始化
void Connection_Init(void);             //上位机TCP连接初始化
void ZF_Assistance_Init(void);          //逐飞助手初始化
void Msg_Process(uint8* rxbuf);         //接收字符串处理函数
void Receive_msg(void);                 //接收字符串
//void Parameter_Debug(void);             //逐飞助手参数调试功能
void PID_display_init(void);            //PID显示参数初始化
void LCD_display(void);                 //LCD显示参数
void Copy_Image_And_Line(void);         //获取显示的图像和边线
void Record_point(uint8 row,uint8 col,uint8 mode); //记录需要显示的标记点

#endif
