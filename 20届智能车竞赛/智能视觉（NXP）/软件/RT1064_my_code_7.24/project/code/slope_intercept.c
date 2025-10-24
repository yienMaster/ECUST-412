#include "image.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include "math.h"

/** 
* @brief 最小二乘法计算斜率
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
* @see		Slope_Calculate(start, end, border);
* @return result					求出的斜率
* ------>x axis
* |
* |
* V
* y axis
*/
AT_ITCM_SECTION_INIT(float Slope_Calculate(uint8 begin_, uint8 end_, uint8 *border))
{
	uint8 begin,end;
	begin = begin_;
	end = end_;
	if(begin > end)
	{
		uint8 temp;
		temp = end;
		end = begin;
		begin = temp;
	}

	float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
	int16 i = 0;
	float result = 0;
	static float resultlast;

	for (i = begin; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		xysum += i * (border[i]);
		x2sum += i * i;

	}
	if ((end - begin)*x2sum - xsum * xsum) //判断除数是否为零
	{
		result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
		resultlast = result;
	}
	else
	{
		result = resultlast;
	}
	return result;
}

/** 
* @brief 最小二乘法计算斜率
* @param uint8 begin				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界首地址
* @see		Slope_Calculate(start, end, border);
* @return result					求出的斜率
* ------>y axis
* |
* |
* V
* x axis
*/
AT_ITCM_SECTION_INIT(float Slope_Calculate_xychange(uint8 begin_, uint8 end_, uint8 *border))
{
	uint8 start,end;
	start = begin_;
	end = end_;
	if(start > end)
	{
		uint8 temp;
		temp = end;
		end = start;
		start = temp;
	}
	float sum_x = 0, sum_y = 0;
  float sum_xy = 0, sum_xx = 0;
  int n = end - start + 1;
    
	for(int i = start; i <= end; i++) {
			float x = border[i];  // x坐标
			float y = image_h - i;  // 转换y坐标，使y轴向上
			sum_x += x;
			sum_y += y;
			sum_xy += x * y;
			sum_xx += x * x;
	}
	
	float mean_x = sum_x / n;
	float mean_y = sum_y / n;
	
	// 计算斜率
	float numerator = sum_xy - sum_x * sum_y / n;
	float denominator = sum_xx - sum_x * sum_x / n;
	
	if(fabs(denominator) < 1e-6) {
			return 0;  // 处理垂直线情况
	}
	
	return numerator / denominator;
}

/** 
* @brief 计算斜率截距
* @param uint8 start				输入起点
* @param uint8 end					输入终点
* @param uint8 *border				输入需要计算斜率的边界
* @param float *slope_rate			输入斜率地址
* @param float *intercept			输入截距地址
* @see 		calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return void
*/
AT_ITCM_SECTION_INIT(void calculate_s_i(uint8 start_, uint8 end_, uint8 *border, float *slope_rate, float *intercept))
{
	uint8 start,end;
	start = start_;
	end = end_;
	if(start > end)
	{
		uint8 temp;
		temp = end;
		end = start;
		start = temp;
	}
	uint16 i, num = 0;
	uint16 xsum = 0, ysum = 0;
	float y_average, x_average;

	num = 0;
	xsum = 0;
	ysum = 0;
	y_average = 0;
	x_average = 0;
	for (i = start; i < end; i++)
	{
		xsum += i;
		ysum += border[i];
		num++;
	}

	//计算各个平均数
	if (num)
	{
		x_average = (float)(xsum / num);
		y_average = (float)(ysum / num);

	}

	/*计算斜率*/
	*slope_rate = Slope_Calculate(start, end, border);//斜率
	*intercept = y_average - (*slope_rate)*x_average;//截距
}

float calculate_slope_improved(uint8 *border, uint8 begin, uint8 end) {
    // 转换坐标系，使y轴向上
    float sum_x = 0, sum_y = 0;
    float sum_xy = 0, sum_xx = 0;
    int n = end - begin + 1;
    
    for(int i = begin; i <= end; i++) {
        float x = border[i];  // x坐标
        float y = image_h - i;  // 转换y坐标，使y轴向上
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_xx += x * x;
    }
    
    float mean_x = sum_x / n;
    float mean_y = sum_y / n;
    
    // 计算斜率
    float numerator = sum_xy - sum_x * sum_y / n;
    float denominator = sum_xx - sum_x * sum_x / n;
    
    if(fabs(denominator) < 1e-6) {
        return 0;  // 处理垂直线情况
    }
    
    return numerator / denominator;
}
