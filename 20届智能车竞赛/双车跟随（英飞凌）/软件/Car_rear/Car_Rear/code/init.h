#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

#define USE_UPPER_COMPUTER          (0)         //是否使用上位机
#define USE_LCD                     (1)         //是否使用LCD


extern uint8 record_time_flag;

void CPU0_Init(void);           //CPU0初始化
void CPU1_Init(void);           //CPU1初始化
void Time_Record_Start(void);   //计时启动
void Time_Record_End(void);     //计时结束

#endif
