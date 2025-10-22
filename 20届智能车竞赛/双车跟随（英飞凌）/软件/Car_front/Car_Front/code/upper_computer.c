#include "upper_computer.h"



uint8 image_display[MT9V03X_H][MT9V03X_W];  //灰度图显示数组
uint8 binary_image_display[MT9V03X_H][MT9V03X_W];   //二值化图显示数组
PID_Display_Structure motor_l_display,motor_r_display,dir_display;  //PID显示参数
float kp2_display,gkd_display;
Boundary_Display_Structure boundary_l_display,boundary_r_display,boundary_m_display;    //边界显示参数
uint8 rxbuf[100];   //接收缓冲区
uint8 image_send_flag=0;    //图传标志位
uint8 para_update_flag=0;   //参数更新标志位
uint8 para_update_display_flag=1;   //参数更新显示标志位
uint8 point_index=MT9V03X_H,point_index_display;    //图传标记点序号起始值
int16 point_display_offset_row[9]={0,0,-1,-1,-1,0,1,1,1};
int16 point_display_offset_col[9]={0,-1,-1,0,1,1,1,0,-1};
float send_buffer_1[100],send_buffer_2[100];
uint16 send_buffer_index=0;
float R2_left_display,R2_right_display;
uint8 test_uint8_display;



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
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, binary_image_display[0], MT9V03X_W, MT9V03X_H);
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
//    else if(!strcmp((char*)variable,"steer")) Steer_Control((uint32)num);
    else if(!strcmp((char*)variable,"motor_l")) Motor_Control(left_motor,(int16)num);
    else if(!strcmp((char*)variable,"motor_r")) Motor_Control(right_motor,(int16)num);
    else if(!strcmp((char*)variable,"target_speed_L")) target_speed_L=num;
    else if(!strcmp((char*)variable,"target_speed_R")) target_speed_R=num;
    else if(!strcmp((char*)variable,"target_speed")) target_speed=num;
    else if(!strcmp((char*)variable,"motor_l_p+")) EEPROM_Write_Data(motor_l_p,motor_l.p+num);
    else if(!strcmp((char*)variable,"motor_l_i+")) EEPROM_Write_Data(motor_l_i,motor_l.i+num);
    else if(!strcmp((char*)variable,"motor_l_p-")) EEPROM_Write_Data(motor_l_p,motor_l.p-num);
    else if(!strcmp((char*)variable,"motor_l_i-")) EEPROM_Write_Data(motor_l_i,motor_l.i-num);
    else if(!strcmp((char*)variable,"motor_r_p+")) EEPROM_Write_Data(motor_r_p,motor_r.p+num);
    else if(!strcmp((char*)variable,"motor_r_i+")) EEPROM_Write_Data(motor_r_i,motor_r.i+num);
    else if(!strcmp((char*)variable,"motor_r_p-")) EEPROM_Write_Data(motor_r_p,motor_r.p-num);
    else if(!strcmp((char*)variable,"motor_r_i-")) EEPROM_Write_Data(motor_r_i,motor_r.i-num);
    else if(!strcmp((char*)variable,"start")) start_flag=num;
    else if(!strcmp((char*)variable,"dir_p+")) EEPROM_Write_Data(dir_p,dir.p+num);
    else if(!strcmp((char*)variable,"dir_i+")) EEPROM_Write_Data(dir_i,dir.i+num);
    else if(!strcmp((char*)variable,"dir_d+")) EEPROM_Write_Data(dir_d,dir.d+num);
    else if(!strcmp((char*)variable,"kp2")) EEPROM_Write_Data(dir_kp2,num);
    else if(!strcmp((char*)variable,"gkd")) EEPROM_Write_Data(dir_gkd,num);
    else if(!strcmp((char*)variable,"error_k")) dir_error_k=num;
    else if(!strcmp((char*)variable,"error_b")) dir_error_b=num;
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

//void Parameter_Debug(void)
//{
//    uint8 i;
//    seekfree_assistant_data_analysis();
//    for(i=0;i<SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT;i++)
//    {
//        if(seekfree_assistant_parameter_update_flag[i])
//        {
//            seekfree_assistant_parameter_update_flag[i] = 0;
//            tft180_show_float(30,110,seekfree_assistant_parameter[i],2,2);
//        }
//    }
//}

void PID_display_init(void)
{
    strcpy((char*)motor_l_display.name,"MOTOR_L");
    strcpy((char*)motor_r_display.name,"MOTOR_R");
    strcpy((char*)dir_display.name,"DIR");
}

void LCD_display(void)
{
    uint8 i;
    tft180_show_uint(90,30,(uint8)round_state,1);
    //图像
#if LCD_IMAGE
    tft180_show_gray_image(0,0,(const uint8 *)binary_image_display[0], MT9V03X_W, MT9V03X_H, MT9V03X_W >>1, MT9V03X_H >>1, 0);
    for(i=0;i<MT9V03X_H;i++)
    {
        if(boundary_l_display.col[i]>0 && boundary_l_display.col[i]<MT9V03X_W-1)
            tft180_draw_point(boundary_l_display.col[i]>>1,i>>1,RGB565_BLUE);
        if(boundary_r_display.col[i]>0 && boundary_r_display.col[i]<MT9V03X_W-1)
            tft180_draw_point(boundary_r_display.col[i]>>1,i>>1,RGB565_GREEN);
        if(boundary_m_display.col[i]>0 && boundary_m_display.col[i]<MT9V03X_W-1)
            tft180_draw_point(boundary_m_display.col[i]>>1,i>>1,RGB565_RED);
    }
    for(i=MT9V03X_H;i<point_index_display;i++)
    {
        if(boundary_m_display.col[i]>0 && boundary_m_display.col[i]<MT9V03X_W-1)
            tft180_draw_point(boundary_m_display.col[i]>>1,boundary_m_display.row[i]>>1,RGB565_RED);
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
        //右电机pid参数
        motor_r_display.p=motor_r.p;
        motor_r_display.i=motor_r.i;
        motor_r_display.d=motor_r.d;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*3,(const char*)motor_r_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*4,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*4,motor_r_display.p,2,3);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*5,"i:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*5,motor_r_display.i,2,3);
        //方向环pid参数
        dir_display.p=dir.p;
        dir_display.i=dir.i;
        dir_display.d=dir.d;
        kp2_display=kp2;
        gkd_display=gkd;
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*6,(const char*)dir_display.name);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*7,"p:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*7,dir_display.p,2,3);
        tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*8,"d:");
        tft180_show_float(LCD_PARA_START_X+12,LCD_PARA_START_Y+LCD_OFFSET_Y*8,dir_display.d,2,3);
        tft180_show_string(LCD_PARA_START_X+60,LCD_PARA_START_Y+LCD_OFFSET_Y*7,"kp2:");
        tft180_show_float(LCD_PARA_START_X+84,LCD_PARA_START_Y+LCD_OFFSET_Y*7,kp2_display,2,3);
        tft180_show_string(LCD_PARA_START_X+60,LCD_PARA_START_Y+LCD_OFFSET_Y*8,"gkd:");
        tft180_show_float(LCD_PARA_START_X+84,LCD_PARA_START_Y+LCD_OFFSET_Y*8,gkd_display,2,3);
    }
    //目标速度
    tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*9,"target_L:");
    tft180_show_float(LCD_PARA_START_X+54,LCD_PARA_START_Y+LCD_OFFSET_Y*9,target_speed_L,3,2);
    tft180_show_string(LCD_PARA_START_X,LCD_PARA_START_Y+LCD_OFFSET_Y*10,"target_R:");
    tft180_show_float(LCD_PARA_START_X+54,LCD_PARA_START_Y+LCD_OFFSET_Y*10,target_speed_R,3,2);
}

void Element_State_Display(uint32 element,uint8* arr)
{
    uint8 i;
    for(i=0;i<25;i++)
    {
        if((element>>i)&1==1) arr[i]=WHITE_IMG;
        else arr[i]=BLACK_IMG;
    }
}

void Copy_Image_And_Line(void)
{
    uint8 i,j,arr[25];
    //复制图像
    memcpy(binary_image_display[0], binary_img_data[0], MT9V03X_IMAGE_SIZE);
    //复制边线
    for(i=0;i<MT9V03X_H;i++)
    {
        boundary_l_display.row[i]=i;
        boundary_r_display.row[i]=i;
        boundary_m_display.row[i]=i;
        if(line_left[i]==LINE_INVALID_LEFT)
            boundary_l_display.col[i]=0;
        else
            boundary_l_display.col[i]=(uint8)line_left[i];
        boundary_r_display.col[i]=(uint8)line_right[i];
        boundary_m_display.col[i]=(uint8)line_middle[i];
    }
    //显示元素种类
    switch(element_state)
    {
        case IDLE: Element_State_Display(STATE_IDLE_DISPLAY,arr); break;
        case STRAIGHT: Element_State_Display(STATE_STRAIGHT_DISPLAY,arr); break;
        case CROSSING: Element_State_Display(STATE_CROSSING_DISPLAY,arr); break;
        case ZEBRA: Element_State_Display(STATE_ZEBRA_DISPLAY,arr); break;
        case CURVE:
            if(curve_position==CURVE_L)
                Element_State_Display(STATE_CURVE_L_DISPLAY,arr);
            else
                Element_State_Display(STATE_CURVE_R_DISPLAY,arr);
            break;
        case ROUND:
            if(round_position==ROUND_L)
                Element_State_Display(STATE_ROUND_L_DISPLAY,arr);
            else
                Element_State_Display(STATE_ROUND_R_DISPLAY,arr);
            break;
    }
    for(i=0;i<5;i++)
    {
        for(j=0;j<5;j++)
        {
            binary_image_display[i+1][j+1]=arr[5*i+j];
        }
    }
}

void Record_point(uint8 row,uint8 col,uint8 mode)
{
    uint8 i;
    if(mode==0)
    {
        for(i=0;i<9;i++)
        {
            boundary_m_display.row[point_index]=row+point_display_offset_row[i];
            boundary_m_display.col[point_index]=col+point_display_offset_col[i];
            boundary_l_display.row[point_index]=0;
            boundary_l_display.col[point_index]=0;
            boundary_r_display.row[point_index]=0;
            boundary_r_display.col[point_index]=MT9V03X_W;
            point_index++;
        }
    }
    else
    {
        for(i=1;i<9;i++)
        {
            boundary_m_display.row[point_index]=row+point_display_offset_row[i];
            boundary_m_display.col[point_index]=col+point_display_offset_col[i];
            boundary_l_display.row[point_index]=0;
            boundary_l_display.col[point_index]=0;
            boundary_r_display.row[point_index]=0;
            boundary_r_display.col[point_index]=MT9V03X_W;
            point_index++;
        }
    }
}




