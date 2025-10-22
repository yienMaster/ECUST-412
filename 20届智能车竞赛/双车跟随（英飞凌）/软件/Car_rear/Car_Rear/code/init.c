#include "init.h"


uint8 record_time_flag=1;   //时间记录标志位

void CPU0_Init(void)
{
    //PID初始化
    PID_Init(&motor_l);
    PID_Init(&motor_r);
    PID_Init(&dir);
    PID_Init(&target_speed_pid);
    //PID参数配置
    PID_Config(&motor_l,
            EEPROM_Read_Data(motor_l_p),
            EEPROM_Read_Data(motor_l_i),
            EEPROM_Read_Data(motor_l_d),
            MOTOR_L_I_MAX,
            MOTOR_L_OUT_MAX,
            MOTOR_L_OUT_MIN);
    PID_Config(&motor_r,
            EEPROM_Read_Data(motor_r_p),
            EEPROM_Read_Data(motor_r_i),
            EEPROM_Read_Data(motor_r_d),
            MOTOR_R_I_MAX,
            MOTOR_R_OUT_MAX,
            MOTOR_R_OUT_MIN);
    PID_Config(&dir,
            EEPROM_Read_Data(dir_p),
            EEPROM_Read_Data(dir_i),
            EEPROM_Read_Data(dir_d),
            DIR_I_MAX,
            DIR_OUT_MAX,
            DIR_OUT_MIN);
    PID_Config(&target_speed_pid,
            EEPROM_Read_Data(target_speed_p),
            EEPROM_Read_Data(target_speed_i),
            EEPROM_Read_Data(target_speed_d),
            TARGET_SPEED_I_MAX,
            TARGET_SPEED_OUT_MAX,
            TARGET_SPEED_OUT_MIN);
    Speed_Para_Update();
    //摄像头初始化
    mt9v03x_init();
    //上位机相关初始化
#if USE_UPPER_COMPUTER
    WIFI_Init();
    Connection_Init();
    ZF_Assistance_Init();
#endif
    //LCD相关初始化
#if USE_LCD
    tft180_set_dir(TFT180_PORTAIT_180);
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_BLACK,RGB565_WHITE);
    tft180_init();
    tft180_show_string(0, 0, "mt9v03x init.");
    PID_display_init();
#endif
    //定时器初始化
    pit_ms_init(CCU60_CH0, 5);
    pit_ms_init(CCU61_CH1, 10);
    pit_ms_init(CCU60_CH1,100);
}

void CPU1_Init(void)
{
    Steer_Init();
    Motor_Init();
    Bldc_Init();
    Encoder_Init();
    Led_Init();
    Buzzer_Init();
    Key_Init();
//    Speed_Para_Update();
}

void Time_Record_Start(void)
{
    if(record_time_flag) system_start();
}

void Time_Record_End(void)
{
    float time;
    if(record_time_flag)
    {
        time=system_getval()/100000.0;
        record_time_flag=0;
#if USE_LCD
        tft180_show_float(0,113,time,2,2);
#endif
    }
}


