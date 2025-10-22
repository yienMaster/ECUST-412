#include "mathfunc.h"



float Constrain_float(float data, float max, float min)
{
    return data > max ? max : (data < min ? min : data);
}

int16 Constrain_int16(int16 data, int16 max, int16 min)
{
    return data > max ? max : (data < min ? min : data);
}

int32 Constrain_int32(int32 data, int32 max, int32 min)
{
    return data > max ? max : (data < min ? min : data);
}

float Average_Filter(int16* arr, int16 data, int16 num)
{
    uint8 i;
    float ave=0;
    for(i=0;i<num-1;i++)
    {
        arr[i]=arr[i+1];
        ave+=arr[i];
    }
    arr[num-1]=data;
    ave+=arr[num-1];
    return ave/num;
}

float str2num(uint8* str)
{
    if (str == NULL) {
        return 0.0f;
    }
    return atof((const char*)str);
}


float Calculate_Slope(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2)
{
    return (float)(col_1-col_2)/(row_1-row_2);
}




