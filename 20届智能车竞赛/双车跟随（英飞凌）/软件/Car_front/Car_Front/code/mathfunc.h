#ifndef __MATHFUNC_H__
#define __MATHFUNC_H__

#include "zf_common_headfile.h"

#define Max(x, y)               (((x) > (y)) ? (x) : (y))                   //���ֵ
#define Min(x, y)               (((x) < (y)) ? (x) : (y))                   //��Сֵ
#define Abs(x)                  ((x) > 0 ? (+(x)) : (-(x)))                 //����ֵ
#define Sign(x)                 ((x) > 0 ? (+1) : ((x) < 0 ? (-1) : 0))     //���ź���
#define Floor(x)                ((int32)(x))                                //��ȡ��
#define Ceil(x)                 ((int32)(x) + 1)                            //��ȡ��
#define LowPass(now, ave, a)    ((now) * (a) + (ave) * (1-(a)))             //��ͨ�˲�

//�޷�����
float Constrain_float(float data, float max, float min);
int16 Constrain_int16(int16 data, int16 max, int16 min);
int32 Constrain_int32(int32 data, int32 max, int32 min);

//��ֵ�˲�
float Average_Filter(int16* arr, int16 data, int16 num);

//�ַ���ת����
float str2num(uint8* str);

//����б��
float Calculate_Slope(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2);

#endif
