#include "upper_computer.h"



uint8 image_display[MT9V03X_H][MT9V03X_W];  //灰度图显示数组
Point_Display_Structure point_u_display,point_d_display,point_m_display;
PID_Display_Structure motor_l_display,motor_r_display,dir_display,target_speed_display;  //PID显示参数
uint8 rxbuf[100];   //接收缓冲区
uint8 image_send_flag=0;    //图传标志位
uint8 para_update_flag=0;   //参数更新标志位
uint8 para_update_display_flag=1;   //参数更新标志位
int16 point_display_offset_row[16]={-2,-2,-2,-2,-2,-1,0,1,2,2,2,2,2,1,0,-1};
int16 point_display_offset_col[16]={-2,-1,0,1,2,2,2,2,2,1,0,-1,-2,-2,-2,-2};
float send_buffer_1[100],send_buffer_2[100];
uint16 send_buffer_index=0;


void WIFI_Init(void)
{
    while(wifi_spi_init(SSID, PASSWORD))
    {
        printf("\r\n connect wifi failed. \r\n");
        system_delay_ms(100);                                                   // 初始化失败 等待 100ms
    }
}

void Connection_Init(void)
{
    while(wifi_spi_socket_connect(                                          // 向指定目标 IP 的端口建立 TCP 连接
        "TCP",                                                              // 指定使用TCP方式通讯
        TARGET_IP,                                                      // 指定远端的IP地址，填写上位机的IP地址
        TARGET_PORT,                                                    // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
        SOURCE_PORT))                                                   // 指定本机的端口号
    {
        // 如果一直建立失败 考虑一下是不是没有接硬件复位
        printf("\r\n Connect TCP Servers error, try again.");
        system_delay_ms(100);                                               // 建立连接失败 等待 100ms
    }
}

void ZF_Assistance_Init(void)
{
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_oscilloscope_data.channel_num = 8;
#if UPPER_COMPUTER_IMAGE
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_display[0], MT9V03X_W, MT9V03X_H);
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, 16, point_u_display.col, point_m_display.col, point_d_display.col, point_u_display.row, point_m_display.row, point_d_display.row);
#endif
}

void Msg_Process(uint8* rxbuf)
{
    uint8 i=0,temp=0;
    uint8 variable[50]={0},data[50]={0};
    float num=0;
    while(rxbuf[i]!='=' && rxbuf[i]!=':')    //等号分隔变量名和值
    {
        variable[i]=rxbuf[i];
        i++;
    }
    i++; temp=i;
    while(rxbuf[i]!='\r' && rxbuf[i]!='\n')
    {
        data[i-temp]=rxbuf[i];
        i++;
    }
    num=str2num(data);
    if(!strcmp((char*)variable,"motor_l_p")) EEPROM_Write_Data(motor_l_p,num);
    else if(!strcmp((char*)variable,"motor_l_i")) EEPROM_Write_Data(motor_l_i,num);
    else if(!strcmp((char*)variable,"motor_l_d")) EEPROM_Write_Data(motor_l_d,num);
    else if(!strcmp((char*)variable,"motor_r_p")) EEPROM_Write_Data(motor_r_p,num);
    else if(!strcmp((char*)variable,"motor_r_i")) EEPROM_Write_Data(motor_r_i,num);
    else if(!strcmp((char*)variable,"motor_r_d")) EEPROM_Write_Data(motor_r_d,num);
    else if(!strcmp((char*)variable,"dir_p")) EEPROM_Write_Data(dir_p,num);
    else if(!strcmp((char*)variable,"dir_i")) EEPROM_Write_Data(dir_i,num);
    else if(!strcmp((char*)variable,"dir_d")) EEPROM_Write_Data(dir_d,num);
    else if(!strcmp((char*)variable,"target_speed_p")) EEPROM_Write_Data(target_speed_p,num);
    else if(!strcmp((char*)variable,"target_speed_i")) EEPROM_Write_Data(target_speed_i,num);
    else if(!strcmp((char*)variable,"target_speed_d")) EEPROM_Write_Data(target_speed_d,num);
    else if(!strcmp((char*)variable,"steer")) Steer_Control((int16)num);
    else if(!strcmp((char*)variable,"motor_l")) Motor_Control(left_motor,(int16)num);
    else if(!strcmp((char*)variable,"motor_r")) Motor_Control(right_motor,(int16)num);
    else if(!strcmp((char*)variable,"target_speed_L")) target_speed_L=num;
    else if(!strcmp((char*)variable,"target_speed_R")) target_speed_R=num;
    else if(!strcmp((char*)variable,"target_speed")) target_speed_L=target_speed_R=num;
    else if(!strcmp((char*)variable,"motor_l_p+")) EEPROM_Write_Data(motor_l_p,motor_l.p+num);
    else if(!strcmp((char*)variable,"motor_l_i+")) EEPROM_Write_Data(motor_l_i,motor_l.i+num);
    else if(!strcmp((char*)variable,"motor_l_p-")) EEPROM_Write_Data(motor_l_p,motor_l.p-num);
    else if(!strcmp((char*)variable,"motor_l_i-")) EEPROM_Write_Data(motor_l_i,motor_l.i-num);
    else if(!strcmp((char*)variable,"motor_r_p+")) EEPROM_Write_Data(motor_r_p,motor_r.p+num);
    else if(!strcmp((char*)variable,"motor_r_i+")) EEPROM_Write_Data(motor_r_i,motor_r.i+num);
    else if(!strcmp((char*)variable,"motor_r_p-")) EEPROM_Write_Data(motor_r_p,motor_r.p-num);
    else if(!strcmp((char*)variable,"motor_r_i-")) EEPROM_Write_Data(motor_r_i,motor_r.i-num);
    else if(!strcmp((char*)variable,"start")) start_flag=num;
    else if(!strcmp((char*)variable,"target_offset")) target_speed_offset_coef=num;
    else if(!strcmp((char*)variable,"bldc")) {BLDC_L(num);BLDC_R(num);}
    para_update_flag=1;
    para_update_display_flag=1;
}

void Receive_msg(void)
{
    wifi_spi_read_buffer(rxbuf,sizeof(rxbuf));  //读缓冲区
    if(strlen((char*)rxbuf)!=0) Msg_Process(rxbuf); //缓冲区有内容
    memset(rxbuf,0,sizeof(rxbuf));  //清空缓冲区
}

void PID_display_init(void)
{
    strcpy((char*)motor_l_display.name,"MOTOR_L");
    strcpy((char*)motor_r_display.name,"MOTOR_R");
    strcpy((char*)dir_display.name,"DIR");
    strcpy((char*)target_speed_display.name,"TARGET");
}

void LCD_display(void)
{
    uint8 i;
    //图像
#if LCD_IMAGE
    tft180_show_gray_image(0,0,(const uint8 *)image_display[0], MT9V03X_W, MT9V03X_H, MT9V03X_W >>1, MT9V03X_H >>1, 0);
    for(i=0;i<16;i++)
    {
        tft180_draw_point(point_u_display.col[i]>>1,point_u_display.row[i]>>1,RGB565_BLUE);
        tft180_draw_point(point_d_display.col[i]>>1,point_d_display.row[i]>>1,RGB565_GREEN);
        tft180_draw_point(point_m_display.col[i]>>1,point_m_display.row[i]>>1,RGB565_RED);
    }
#endif
    if(para_update_display_flag  && !para_update_flag)
    {
        para_update_display_flag=0;
        //左电机pid参数
        motor_l_display.p=motor_l.p;
        motor_l_display.i=motor_l.i;
        motor_l_display.d=motor_l.d;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y,(const char*)motor_l_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y,motor_l_display.p,2,3);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*2,"i:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*2,motor_l_display.i,2,3);
//        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*3,"d:");
//        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*3,motor_l_display.d,2,2);
        //右电机pid参数
        motor_r_display.p=motor_r.p;
        motor_r_display.i=motor_r.i;
        motor_r_display.d=motor_r.d;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*3,(const char*)motor_r_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*4,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*4,motor_r_display.p,2,3);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*5,"i:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*5,motor_r_display.i,2,3);
//        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*6,"d:");
//        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*6,motor_r_display.d,2,2);
        //方向环pid参数
        dir_display.p=dir.p;
        dir_display.i=dir.i;
        dir_display.d=dir.d;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*6,(const char*)dir_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*7,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*7,dir_display.p,3,3);
//        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*8,"i:");
//        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*8,dir_display.i,2,2);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*8,"d:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*8,dir_display.d,3,3);
        //目标速度环pid参数
        target_speed_display.p=target_speed_pid.p;
        target_speed_display.i=target_speed_pid.i;
        target_speed_display.d=target_speed_pid.d;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*9,(const char*)target_speed_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*10,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*10,target_speed_display.p,2,4);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*11,"i:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*11,target_speed_display.i,2,4);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*12,"d:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*12,target_speed_display.d,2,4);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*13,"offset:");
        tft180_show_float(LCD_PARA_START_X+42,LCD_PARA_START_Y+LCD_OFFSET_Y*13,target_speed_offset_coef,1,4);
    }
}

void Copy_Image_And_Point(void)
{
    uint8 i;
    //复制图像
    memcpy(image_display[0], img_data[0], MT9V03X_IMAGE_SIZE);
    //复制点
    for(i=0;i<16;i++)
    {
        point_u_display.col[i]=(uint8)Constrain_int16(point_up.col+point_display_offset_col[i],MT9V03X_W-1,0);
        point_u_display.row[i]=(uint8)Constrain_int16(point_up.row+point_display_offset_row[i],MT9V03X_H-1,0);
        point_m_display.col[i]=(uint8)Constrain_int16(point_middle.col+point_display_offset_col[i],MT9V03X_W-1,0);
        point_m_display.row[i]=(uint8)Constrain_int16(point_middle.row+point_display_offset_row[i],MT9V03X_H-1,0);
        point_d_display.col[i]=(uint8)Constrain_int16(point_down.col+point_display_offset_col[i],MT9V03X_W-1,0);
        point_d_display.row[i]=(uint8)Constrain_int16(point_down.row+point_display_offset_row[i],MT9V03X_H-1,0);
    }
}




