#include "my_code.h"
#include "math.h"
uint8 uart_get_data[64];                                          // 接受数据（数组形式）
uint8 fifo_get_data[64];                                          // 接受数据长度
uint8 get_data = 0;// ??????
uint8 ort_get_data = 0;// ??????
uint8 mcx_z;
uint8 dat;
uint32 fifo_data_count = 0;                                       // fifo ????
fifo_struct uart_data_fifo;
pid_controller motor1_pid;
pid_controller motor2_pid;
pid_controller motor3_pid;

fifo_struct openart_fifo;
uint8 uart_ort_data[64];                                          // 接受数据（数组形式）
uint8 fifo_ort_data[64];
uint32 fifo_ort_data_count = 0;
uint8 openart_data = 0;
int openart_cnt = 0;
uint8 data_uart_1[20];


char mcx_str1[20],mcx_str2[20],mcx_str3[20];
char fs[20];

uint8 ort_info[3];
int fifo_State;

void get_openart_info();
void get_mcx_info(void);
//========================= main ==========================================================
int main(void)
{
	clock_init(SYSTEM_CLOCK_600M);
	debug_init();

//====================== initialize area ==================================================
	//set pit to 1ms priority to 0
	pit_ms_init(PIT_CH, 1); 
	interrupt_set_priority(PIT_PRIORITY, 0);
	
	
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);	//fifo init
	
	fifo_init(&openart_fifo, FIFO_DATA_8BIT, uart_ort_data, 64);	//fifo init
	
	uart_init(UART1_INDEX, UART_BAUDRATE, UART1_TX_PIN, UART1_RX_PIN);  
	uart_rx_interrupt(UART1_INDEX, ZF_ENABLE);
	interrupt_set_priority(UART1_PRIORITY, 2);
	    
	uart_init(UART2_INDEX, UART_BAUDRATE, UART2_TX_PIN, UART2_RX_PIN);
	uart_rx_interrupt(UART2_INDEX, ZF_ENABLE);
	interrupt_set_priority(UART2_PRIORITY, 1);
	//uart_init(UART_4, SCC8660_FLEXIO_COF_BAUR, UART3_TX_PIN, UART3_RX_PIN);
	//uart_rx_interrupt(UART_4, ZF_ENABLE);
	//interrupt_set_priority(UART4_PRIORITY, 4);
	scc8660_flexio_init();
	
	uart_write_string(UART1_INDEX, "UART Text.");                                // 输出测试信息
	uart_write_byte(UART1_INDEX, '\r');                                          // 输出回车
	uart_write_byte(UART1_INDEX, '\n');                                          // 输出换行
	imu660ra_init();
	encoder_init();
	motor_init();
	//openart_mini_init();
	gpio_init(BUZZER_PIN, GPO, 0, GPO_PUSH_PULL);										//buzzer init
	
	//tft180 init
	tft180_set_dir(TFT180_CROSSWISE);
	//tft180_set_dir(TFT180_PORTAIT);
	tft180_init();
	//ips200_init(IPS200_TYPE_PARALLEL8);
	//mt9v03x init
	while(mt9v03x_init())
	{
			//tft180_show_string(0,16,"mt9v03x reinit...");
	}
	system_delay_ms(100);
	//tft180_show_string(0,16,"mt9v03x init success");
	interrupt_global_enable(0);
	
	key_init(10);																										//key init
	
//======================= initialize area ====================================================
	
	system_delay_ms(1000);
	tft180_clear();

	bool translation = false;
	int count = 0;
	int mcx_search_time = 1;
	//===================== pid area =========================
	
	set_pid(7.0f,0.25f,0.0f,&motor1_pid);
	set_pid(7.5f,0.25f,0.01f,&motor2_pid);
	set_pid(8.0f,0.22f,0.007f,&motor3_pid);
	
	set_pid(1.242f, 0.135f, 0.0025f, &cl_angle_pid);
	set_pid(0.5f,0.01f,0.0f,&cl_angle_pid_mini);
	set_pid(0.25f,0.005f,0.00f, &dv_pid);
	set_pid(0.0f,0.03f,0.0f, &mcx_pid);
	
	//dv_pid.output_max = 15;
	//dv_pid.output_min = -15;
	

	//===================== pid area ==========================
	
	
	//================== temps ==============================

	char motor1_vel[20],motor2_vel[20],motor3_vel[20];
	char IMUpitch[20], IMUroll[20], IMUyaw[20];
	char gyro_z[20];
	
	//=======================================================
	
	//clear encoders
	encoder_clear_count(ENCODER1_QUADDEC);
	encoder_clear_count(ENCODER2_QUADDEC);
	encoder_clear_count(ENCODER3_QUADDEC);
	
	while(1)
	{ 
		//ort_show();
		//tft180_show_string(0,50,labels[ort_label[1]]);
		if(pit_flag)
		{
			pit_flag = false;
			
			cnt_1ms += 1;
			if(cnt_1ms >= 100)
			{
				cnt_01s += 1;
				cnt_1ms = 0;
			}
			//tft180_show_int(0, 5, cnt_01s, 5);
			key_detect();
//			imu660ra_get_gyro();
//			imu660ra_get_acc();
			IMU_getEulerianAngles();
			if(mt9v03x_finish_flag)
			{
				//tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W/2, MT9V03X_H/2, 0);
				if(mcx_temp == 1) image_process();
				centerline_follow(center_line, &m_angle, &t_angle);
				if(!zebra_detect)
					tft180_show_gray_image(0, 0, bin_image[0], MT9V03X_W, MT9V03X_H, 160, 128, 0);
				mt9v03x_finish_flag = 0;
			}

		}
		get_mcx_info();
		
		get_openart_info();

		
		//get_openart_info();
		if (zebra_detect == false)
		{
			if (cnt_01s - mcx_cnt_01s == mcx_search_time)
			{
				mcx_find_flag = 0;
				mcx_detect_flag = 0;
				mcx_flag = 0;
				d_count = 0;
				encoder_clear_count(ENCODER1_QUADDEC);
				encoder_clear_count(ENCODER2_QUADDEC);
				encoder_clear_count(ENCODER3_QUADDEC);
			}
			if ((mcx_find_flag == 0 && mcx_detect_flag == 0)||(cnt_01s - mcx_cnt_01s < mcx_search_time))
			{
				mcx_temp = 1;
			}
			else if(mcx_find_flag && !mcx_detect_flag )
			{
				mcx_temp = 2;
				
			}
//			else if(mcx_find_flag && mcx_detect_flag && ort_transport == 0 && mcx_cnt_01s - mcx_time == 3)
//			{
//				ort_transport=1;
//				ort_label[ort_line] = 12;
//				ort_line++;
//				
//			}
			else if(mcx_find_flag && mcx_detect_flag && ort_transport == 1)
			{
				l_or_r = openart_judge((uint8)openart_info);
				if (d_count == 0)
				{
					count = 0;
					d_count = 1;
				}
				if(l_or_r == 0)
					mcx_temp = 3;
			
				else
				{
					mcx_temp = 4;
				}
			}
		}
		else
		{
			mcx_temp = 5;
			tft180_set_dir(TFT180_PORTAIT);
			tft180_clear();
			ort_show();
			//tft180_show_string(0, 20, "stop");
		}
			//get_openart_info();
//			sprintf(mcx_str1,"%d",openart_info);
//			tft180_show_string(0,25,mcx_str1);			
//			sprintf(mcx_str2,"%d",mcx_temp);
//			tft180_show_string(0,65,mcx_str2);
//			sprintf(mcx_str3,"%d",ort_info[2]);
//			tft180_show_string(0,95,mcx_str3);
//			sprintf(fs, "%d", fifo_State);
//			tft180_show_string(0,10,fs);
			
			
		}
		
		//move_ctrl(40, move_angle, turn_angle);
		

			
		
}//==========This is main's '}'==========	

//uart_init(UART4_INDEX, UART_BAUDRATE, UART4_TX_PIN, UART4_RX_PIN);             
//uart_rx_interrupt(UART4_INDEX, ZF_ENABLE);                                   // ?? UART_INDEX ?????
//interrupt_set_priority(UART4_PRIORITY, 1);                                   // ???? UART_INDEX ??????? 0

//uart_write_string(UART4_INDEX, "UART Text.");                                // ??????
//uart_write_byte(UART4_INDEX, '\r');                                          // ????
//uart_write_byte(UART4_INDEX, '\n');                                          // ????

void get_mcx_info(void)
{
	if (!mcx_detect_flag)
	{
			fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
			if(fifo_data_count != 0)                                                // 读取到数据了
			{
				fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
				mcx_flag = fifo_get_data[0]-48;
				mcx_info = 0;
				for(int i = 1; i < 4; i++)
				{
					mcx_info = mcx_info * 10 + (fifo_get_data[i] - 48);
				}
	//						sprintf(mcx_str,"%d",mcx_info);
	//						tft180_show_string(0,0,mcx_str);
				if(mcx_info < 620 && mcx_info > 300)
				{
					//gpio_set_level(BUZZER_PIN,1);
					mcx_info -= 300;
					mcx_pid_count = 0;
					mcx_find_flag = 1;//发现箱子状态标志位
					
					//mcx_detect_flag = 0;//箱子未处于图像中间
				}
			}
		}
}



void get_openart_info()
{
	fifo_ort_data_count = fifo_used(&openart_fifo);                           // 查看 fifo 是否有数据
	if(mcx_detect_flag && mcx_find_flag && ort_transport==0)
	{
		if(fifo_ort_data_count != 0)                                                // 读取到数据了
		{
			openart_info=0;
			fifo_State = fifo_read_buffer(&openart_fifo, fifo_ort_data, &fifo_ort_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
			for(int i=0;i<3;i++)
			{
				if(fifo_ort_data[0] == 0) return;
				openart_info = openart_info * 10 + fifo_ort_data[i] - 48;
				ort_info[i] = fifo_ort_data[i];
			}
			openart_info -= 100;
			if(openart_info>=0 && openart_info<=115)
			{
				ort_transport=1;
				if (ort_line<=11)
				{
					ort_label[ort_line] = openart_info;
					ort_line++;
				}
			}
			//gpio_set_level(BUZZER_PIN, 1);
		}
	}

}
//void get_openart_info()
//{
//	
//	int id_uart1=0;
//	uint8 get_data_1=0;
//	uart_query_byte(UART2_INDEX,&get_data_1);
//	
//	if(get_data_1=='s')
//	{
//		gpio_set_level(BUZZER_PIN, 0);
//		id_uart1=0;
//		return;
//	}
//	if(get_data_1=='e')
//	{
//		data_uart_1[id_uart1]='\0';
//		ort_transport=1;
//		return;
//	}
//	for(int i=1;i<4;i++)
//	{
//		ort_info[i-1] =data_uart_1[i];
//	}
//	data_uart_1[id_uart1]=get_data_1;
//	id_uart1++;
//	
//}

void uart_rx_mcx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
			uart_query_byte(UART1_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
			fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
			
}
void uart_rx_openart_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
			uart_query_byte(UART2_INDEX, &ort_get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
			fifo_write_buffer(&openart_fifo, &ort_get_data, 1);                           // 将数据写入 fifo 中
			if(openart_fifo.size < openart_fifo.max - 3)
			{
				fifo_clear(&openart_fifo);	
				fifo_ort_data[0] = 0;
				fifo_ort_data[1] = 0;
				fifo_ort_data[2] = 0;
			}
}