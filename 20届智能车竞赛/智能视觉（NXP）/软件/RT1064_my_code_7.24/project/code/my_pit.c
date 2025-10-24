#include "my_code.h"
#include <string.h>
bool pit_flag = false;
bool speak = false;   //设置蜂鸣器响的标志位
int my_move_dir = 90;
uint8 m_angle = 90;
int16 t_angle = 0;
int count;
uint8 mcx_find_flag=0;//用于存储mcx是否检测到箱子
int mcx_info=0;//用于存贮mcx发送信息
uint8 mcx_detect_flag=0;//用于判断箱子是否处于图像中间
uint8 mcx_temp;
uint8 mcx_lorr;
uint8 l_or_r = 0;
int16 cnt_1ms = 0;
int16 cnt_01s = 2;
int16 cnt_stop = 0;
/**
 * @brief pit中断时执行的函数, 见isr.c
 */
void pit_handler()
{
	pit_flag = true;
	//move_ctrl(45, m_angle, t_angle);
//	if (cnt_01s - mcx_cnt_01s == 3)
//			{
//				mcx_find_flag = 0;
//				mcx_detect_flag = 0;
//				mcx_flag = 0;
//			}			
	

	switch (mcx_temp)
	{
		case 1: 
			move_ctrl(40,m_angle,t_angle);
			break;//move_ctrl(30,m_angle, t_angle);gpio_set_level(BUZZER_PIN, 0);break;
		case 2: 
			box_detect(mcx_info);
			break;
		case 3: 
			box_pull_right();
			break;
		case 4: 
			box_pull_left();
			break;
		case 5:
			move_ctrl(0,90,0);
			break;
		
	}
//	count++;
//	if(count == 1000)
	
	//encoder1_prev = encoder1_data;
	
	//encoder2_data = get_encoder2_data();
	//encoder3_data = get_encoder3_data();
	
}
