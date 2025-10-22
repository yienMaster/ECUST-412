#ifndef __INIT_H__
#define __INIT_H__

#include "zf_common_headfile.h"

#define USE_UPPER_COMPUTER          (0)         //�Ƿ�ʹ����λ��
#define USE_LCD                     (1)         //�Ƿ�ʹ��LCD


extern uint8 record_time_flag;

void CPU0_Init(void);           //CPU0��ʼ��
void CPU1_Init(void);           //CPU1��ʼ��
void Time_Record_Start(void);   //��ʱ����
void Time_Record_End(void);     //��ʱ����

#endif
