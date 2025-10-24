#include "zf_common_headfile.h"
#include "my_code.h"
//uint8 mcx_find_flag=0;//用于存储mcx是否检测到箱子
//uint8 mcx_info=0;//用于存贮mcx发送信息
//uint8 mcx_detect_flag=0;//用于判断箱子是否处于图像中间
char mcx_data[50];
extern int count;
int d_count = 0;
int16 mcx_turn_angle;
int16 mcx_move_speed=6;
int mcx_flag;//用于判断mcx与箱子直接的距离是否到达目标值
int mcx_pid_count;
int mcx_cnt_01s = 0;
int mcx_time_flag=0;
int mcx_time = 0;
uint8 ort_transport;
char e_da[20];
void mcx_info_get(void)  //mcx发现箱子
{
		mcx_detect_flag= 0;
}


void box_detect(int mcx_dat)//控制小车移动到箱子中间
{

			mcx_pid_count++;
			if(mcx_pid_count%2000==0)
			{
					//mcx_pid_count = 0;
					mcx_flag = 2;
			}
//			if(mcx_pid_count == 20000)
//			{
//				ort_transport=1;
//				ort_label[ort_line] = 32;
//				ort_line++;
//			}
//		if(mcx_pid_count<=10)
//		{
//			move_ctrl(-60,90,0);
//		}
//		if(mcx_pid_count==11)
//		{
//			motor1_pid.e_prev = 0;
//			motor1_pid.d_prev = 0;
//			motor2_pid.e_prev = 0;
//			motor2_pid.d_prev = 0;
//			motor3_pid.e_prev = 0;
//			motor3_pid.d_prev = 0;
//		}
//		else
//		{
			gpio_set_level(BUZZER_PIN,1);
			mcx_turn_angle =90 + 0.5*(160-mcx_dat);
			//mcx_turn_angle = 90+(int)PID_control(0,160-mcx_dat, &mcx_pid);
			//mcx_move_speed =(int)(0.1 * abs(160-mcx_dat));
			
			if(mcx_flag == 2)
			{
				move_ctrl(-mcx_move_speed,90,0);
//				count=0;
//					gpio_set_level(BUZZER_PIN, 0);
//					mcx_detect_flag = 1;
//					mcx_pid_count = 0;
//					move_ctrl(0,90,0);
				
			}
			else if(abs(mcx_dat-160)<= 10)
			{
				if (mcx_flag == 1)
				{
					count=0;
//					if(mcx_time_flag==0)
//					{
//						mcx_time = cnt_01s;
//						mcx_time_flag = 1;
//					}
					gpio_set_level(BUZZER_PIN, 0);
					mcx_detect_flag = 1;
					mcx_pid_count = 0;
					move_ctrl(0,90,0);
//					fifo_clear(&openart_fifo);	
//					fifo_ort_data[0] = 0;
//					fifo_ort_data[1] = 0;
//					fifo_ort_data[2] = 0;
				}
				else
					move_ctrl(mcx_move_speed,mcx_turn_angle,0);
			}
			else
				move_ctrl(mcx_move_speed,mcx_turn_angle,0);
//		}
}
int wait_time = 510;
int horizen = 430;
int spin_r = 110;
int forward = 500;
int backward = 340;
int spin_l = 210;
int wait_time_r = 2000;
void box_pull_right (void)	//推箱子函数
{
	
	count++;
	if(count < wait_time)
	{
		move_ctrl(0,90,0);
		gpio_set_level(BUZZER_PIN, 1);
	
	}
	else if(count<=wait_time + horizen && count >= wait_time)
	{
		tft180_show_string(0, 40, "left");
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(50,130,0);
	}
	else if (count<=wait_time + horizen + spin_r && count > wait_time + horizen)
	{
		tft180_show_string(0, 40, "forw");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(0,90,-60);
	}
	else if (count<=wait_time + horizen + spin_r + forward && count>wait_time + horizen + spin_r)
	{
		tft180_show_string(0, 40, "right");
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(80,90,0);
	}
	else if (count<=wait_time + horizen + spin_r + forward + backward && count > wait_time + horizen + spin_r + forward)
	{
		tft180_show_string(0, 40, "left");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(-80,90,0);
	}
	else if (count<=wait_time + horizen + spin_r + forward + backward + spin_l && count > wait_time + horizen + spin_r + forward + backward )
	{
		tft180_show_string(0, 40, "forw");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(0,90,50);
	}
//	else if (count<=wait_time + horizen + spin_r + forward + backward + spin_l + wait_time_r && count > wait_time + horizen + spin_r + forward + backward)
//	{

//		gpio_set_level(BUZZER_PIN, 0);
//		move_ctrl(0,90,0);
//		
//	}
	else
	{
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(0,90,0);
		encoder_clear_count(ENCODER1_QUADDEC);
		encoder_clear_count(ENCODER2_QUADDEC);
		encoder_clear_count(ENCODER3_QUADDEC);
		mcx_find_flag = 0;
		mcx_detect_flag = 0;
		mcx_flag = 0;
		mcx_cnt_01s = cnt_01s;
		mcx_pid_count=0;
		set_pid(7.0f,0.25f,0.0f,&motor1_pid);
		set_pid(7.5f,0.25f,0.01f,&motor2_pid);
		set_pid(8.0f,0.22f,0.007f,&motor3_pid);
		
		set_pid(1.25f, 0.135f, 0.0025f, &cl_angle_pid);
		set_pid(0.5f,0.01f,0.0f,&cl_angle_pid_mini);
		set_pid(0.25f,0.005f,0.00f, &dv_pid);
		set_pid(0.0f,0.03f,0.0f, &mcx_pid);
		ort_transport=0;
		d_count = 0;
		openart_info = -1;
		//mcx_time_flag=0;
	}
}
int wait_time_l = 500;
int horizen_l = 425;
int spin_r_l = 80;
int forward_l = 500;
int backward_l = 360;
int spin_l_l = 200;
int back_l=2000;
void box_pull_left (void)	//推箱子函数
{
	
	count++;
	if(count < wait_time)
	{
		set_motor1_vel(0);
		set_motor2_vel(0);
		set_motor3_vel(0);
		gpio_set_level(BUZZER_PIN, 0);
	}
	else if(count<=wait_time_l + horizen_l && count >= wait_time_l)
	{
		tft180_show_string(0, 40, "left");
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(50,45,0);
	}
	else if (count<=wait_time_l + horizen_l + spin_r_l && count > wait_time_l + horizen_l)
	{
		tft180_show_string(0, 40, "forw");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(0,90,80);
	}
	else if (count<=wait_time_l + horizen_l + spin_r_l + forward_l&& count>wait_time_l + horizen_l + spin_r_l)
	{
		tft180_show_string(0, 40, "right");
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(80,90,0);
	}
	else if (count<=wait_time_l + horizen_l + spin_r_l + forward_l + backward_l && count > wait_time_l + horizen_l + spin_r_l + forward_l)
	{
		tft180_show_string(0, 40, "left");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(-80,90,0);
	}
	else if (count<=wait_time + horizen_l + spin_r_l + forward_l + backward_l + spin_l_l && count > wait_time_l + horizen_l + spin_r_l + forward_l + backward_l )
	{
		tft180_show_string(0, 40, "forw");
		gpio_set_level(BUZZER_PIN, 1);
		move_ctrl(0,90,-50);
	}
//	else if (count<=wait_time + horizen_l + spin_r_l + forward_l + backward_l + spin_l_l + back_l && count > wait_time_l + horizen_l + spin_r_l + forward_l + backward_l+ spin_l_l )
//	{
//		
//		tft180_show_string(0, 40, "forw");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(0,90,0);
//	}
	else 
	{
		gpio_set_level(BUZZER_PIN, 0);
		move_ctrl(0,90,0);
		encoder_clear_count(ENCODER1_QUADDEC);
		encoder_clear_count(ENCODER2_QUADDEC);
		encoder_clear_count(ENCODER3_QUADDEC);
		mcx_find_flag = 0;
		mcx_detect_flag = 0;
		mcx_flag = 0;
		mcx_cnt_01s = cnt_01s;
		mcx_pid_count=0;
		set_pid(7.0f,0.25f,0.0f,&motor1_pid);
		set_pid(7.5f,0.25f,0.01f,&motor2_pid);
		set_pid(8.0f,0.22f,0.007f,&motor3_pid);
		
		set_pid(1.25f, 0.135f, 0.0025f, &cl_angle_pid);
		set_pid(0.5f,0.01f,0.0f,&cl_angle_pid_mini);
		set_pid(0.25f,0.005f,0.00f, &dv_pid);
		set_pid(0.0f,0.03f,0.0f, &mcx_pid);
		ort_transport=0;
		d_count = 0;
		//mcx_time_flag=0;
		openart_info = -1;
	}
}
void box_pull(int l_or_r)
{
	if(l_or_r==0)
	{
		box_pull_right();
	}
	else
	{
		box_pull_left();
	}
}
//long long int time1=15000,time2=15000+600,time3=15000+600+40000,time4=15000+600+40000+40000,time5=15000+600+40000+40000+20000;

//void box_pull_right (void)	//推箱子函数
//{
//	count++;
//	if(count < wait_time)
//	{
//		set_motor1_vel(0);
//		set_motor2_vel(0);
//		set_motor3_vel(0);
//		//gpio_set_level(BUZZER_PIN, 1);
//	}
//	else
//	{
//		encoder_timer+=abs(encoder3_data);
//	}
//	if(encoder_timer<time1)//&& count> wait_time
//	{
//		tft180_show_string(0, 40, "left");
//		gpio_set_level(BUZZER_PIN, 0);
//		move_ctrl(50,135,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time1)
//			encoder_timer = time1;
//	}
//	else if (encoder_timer<time2 && encoder_timer > time1)
//	{
//		tft180_show_string(0, 40, "forw");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(0,90,-50);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time2)
//			encoder_timer = time2;
//	}
//	else if (encoder_timer<time3 && encoder_timer>time2)
//	{
//		tft180_show_string(0, 40, "right");
//		gpio_set_level(BUZZER_PIN, 0);
//		move_ctrl(50,90,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time3)
//			encoder_timer = time3;
//	}
//	else if (encoder_timer<time4 && encoder_timer >time3)
//	{
//		tft180_show_string(0, 40, "left");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(-50,90,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time4)
//			encoder_timer = time4;
//	}
//	else if (encoder_timer<time5 && encoder_timer > time4 )
//	{
//		tft180_show_string(0, 40, "forw");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(0,90,50);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time5)
//			encoder_timer = time5;
//	}
//	else
//	{
//		gpio_set_level(BUZZER_PIN, 0);
//		set_motor1_vel(0);
//		set_motor2_vel(0);
//		set_motor3_vel(0);
//		sprintf(e_da,"%d",encoder_timer);
//		tft180_show_string(0,0,e_da);
//		//mcx_detect_flag = 0;
//	}
//	
//}

//long long int l_time1=8000,l_time2=8000+8000,l_time3=8000+8000+80000,l_time4=8000+8000+80000+40000,l_time5=8000+8000+80000+40000+18000;
//void box_pull_left (void)	//推箱子函数
//{
//	
//	count++;
//	if(count < wait_time)
//	{
//		set_motor1_vel(0);
//		set_motor2_vel(0);
//		set_motor3_vel(0);
//		gpio_set_level(BUZZER_PIN, 1);
//	}
//	else
//	{
//		encoder_timer+=abs(encoder3_data);
//	}
//	if(encoder_timer<l_time1)
//	{
//		tft180_show_string(0, 40, "left");
//		gpio_set_level(BUZZER_PIN, 0);
//		move_ctrl(50,130,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time1)
//			encoder_timer = time1;
//	}
//	else if (encoder_timer<l_time2 && encoder_timer > l_time1)
//	{
//		tft180_show_string(0, 40, "forw");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(0,90,-50);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time2)
//			encoder_timer = time2;
//	}
//	else if (encoder_timer<l_time3 && encoder_timer>l_time2)
//	{
//		tft180_show_string(0, 40, "right");
//		gpio_set_level(BUZZER_PIN, 0);
//		move_ctrl(50,90,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time3)
//			encoder_timer = time3;
//	}
//	else if (encoder_timer<l_time4 && encoder_timer >l_time3)
//	{
//		tft180_show_string(0, 40, "left");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(-50,90,0);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time4)
//			encoder_timer = time4;
//	}
//	else if (encoder_timer<l_time5 && encoder_timer > l_time4 )
//	{
//		tft180_show_string(0, 40, "forw");
//		gpio_set_level(BUZZER_PIN, 1);
//		move_ctrl(0,90,50);
//		encoder_timer+=abs(encoder3_data);
//		if(encoder_timer>time5)
//			encoder_timer = time5;
//	}
//	else
//	{
//		gpio_set_level(BUZZER_PIN, 0);
//		set_motor1_vel(0);
//		set_motor2_vel(0);
//		set_motor3_vel(0);
//		sprintf(e_da,"%d",encoder_timer);
//		tft180_show_string(0,0,e_da);
//		//mcx_detect_flag = 0;
//	}
//	
//}
