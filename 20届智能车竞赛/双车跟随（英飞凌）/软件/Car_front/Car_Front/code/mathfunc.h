#ifndef __MATHFUNC_H__
#define __MATHFUNC_H__

#include "zf_common_headfile.h"

#define Max(x, y)               (((x) > (y)) ? (x) : (y))                   //最大值
#define Min(x, y)               (((x) < (y)) ? (x) : (y))                   //最小值
#define Abs(x)                  ((x) > 0 ? (+(x)) : (-(x)))                 //绝对值
#define Sign(x)                 ((x) > 0 ? (+1) : ((x) < 0 ? (-1) : 0))     //符号函数
#define Floor(x)                ((int32)(x))                                //下取整
#define Ceil(x)                 ((int32)(x) + 1)                            //上取整
#define LowPass(now, ave, a)    ((now) * (a) + (ave) * (1-(a)))             //低通滤波

//限幅函数
float Constrain_float(float data, float max, float min);
int16 Constrain_int16(int16 data, int16 max, int16 min);
int32 Constrain_int32(int32 data, int32 max, int32 min);

//均值滤波
float Average_Filter(int16* arr, int16 data, int16 num);

//字符串转数字
float str2num(uint8* str);

//计算斜率
float Calculate_Slope(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2);

#endif
