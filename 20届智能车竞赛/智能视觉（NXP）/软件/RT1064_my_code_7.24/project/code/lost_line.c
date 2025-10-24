#include "image.h"
#include "slope_intercept.h"
#include "zf_common_headfile.h"
#include "math.h"
/**
 * @brief 计算数组指定范围内的平均值
 * @param arr 输入数组
 * @param _start 起始索引
 * @param _end 结束索引
 * @return float 计算得到的平均值
 */
float arr_average(uint16* arr, uint8 _start, uint8 _end)
{
	// 参数有效性检查
	if (arr == NULL) 
	{
		return 0.0f;
	}
	
	// 确保start <= end
	uint8 start = (_start <= _end) ? _start : _end;
	uint8 end = (_start <= _end) ? _end : _start;
	
	// 计算元素个数
	uint8 count = end - start + 1;
	if (count == 0) {
		return 0.0f;
	}
	
	// 使用double类型进行累加，避免精度损失
	double sum = 0.0;
	for (uint8 i = start; i <= end; i++) {
		sum += (double)arr[i];
	}
	
	return (float)(sum / count);
}


AT_ITCM_SECTION_INIT(void fill_line(uint8 *l, uint8 *r, unsigned char(*p)[image_w], unsigned short *indexl, unsigned short *indexr))
{
	unsigned char h = 0;
	
	/*
		过圆环内环
	*/
	int16 opl = 0;
	for (h = image_h - 15; h > 5; h--)
	{
		if (l[h] > l[h + 5] && l[h] > l[h - 5] && l[h - 5] > border_min + 2)
		{
			opl = h;
			break;
		}
	}
	if (opl)
	{
		for(h = opl + 10; h > opl - 60; h--)
		{
			if(r[h] < r[h + 1] && r[h + 1] - r[h] > 10)
			{
				opl = 0;
				break;
			}
		}
	}
	if (opl)
	{
		float k = 0, b = 0;
		k = ((float)(l[image_h - 1] - l[opl]) / (float)(image_h - 1 - opl));
		b = (float)(l[opl] - opl * k);
		int temp = 0;
		for (unsigned char i = opl; i < image_h - 1; i++)
		{
			temp = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(temp, border_min, border_max);//限幅
		}
		tft180_show_string(0, 50, "opl");
		tft180_show_int(0,65,opl, 3);
	}

	int16 opr = 0;
	for (h = image_h - 15; h > 5; h--)
	{
		if (r[h] < r[h + 5] && r[h] < r[h - 5] && r[h - 5] < border_max - 2)
		{
			opr = h;
			break;
		}
	}
	if (opr)
	{
		for(h = opr + 10; h > opr - 60; h--)
		{
			if(l[h] > l[h + 1] && l[h] - l[h + 1] > 10)
			{
				opr = 0;
				break;
			}
			
		}
	}
	if (opr)
	{
		float k = 0, b = 0;
		k = ((float)(r[image_h - 1] - r[opr]) / (float)(image_h - 1 - opr));
		b = (float)(r[opr] - opr * k);
		int temp = 0;
		for (unsigned char i = opr; i < image_h - 1; i++)
		{
			temp = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(temp, border_min, border_max);//限幅
		}
		tft180_show_string(0, 50, "opr");
		tft180_show_int(0,65,opr, 3);
	}

	if(opl && opr) return;
		
	/*
		补十字
	  检测十字的方式是看前半段的拐点
	*/
	uint8 templ = 0;
	for(h = image_h - 6; h > 60; h--)
	{
		if((l[h] - l [h - 3] > 10 || l[h - 3] < border_min + 2) && l[h + 3] < l[h])
		{		
			templ = h;
			break;
		}
		
	}
	
	uint8 tempr = 0;
	for(h = image_h - 6; h > 60; h--)
	{
		if((r[h - 3] - r[h] > 10 || r[h - 3] > border_max - 2) && r[h + 5] > r[h])
		{		
			tempr = h;
			break;
		}
		
	}
	if(templ && tempr)
	{		
		for(uint8 i = templ; i > templ - 60; i--)
		{
			if(l[i] < border_min + 2)
			{
				l[i] = border_min + 35;
			}
			
		}
		for(uint8 i = tempr; i > tempr - 60; i--)
		{
			if(r[i] > border_max - 2)
			{
				
				r[i] = border_max - 35;
			}
		}
	}
	/*
		出圆环或者补十字
	  检测十字的方式是看后半段的拐点
	*/
	uint8 temphl = 0;
	for (h = image_h - 9; h > 20; h--)
	{
		//如果左线向左前方, 而且差值大于15
		if (l[h] > l[h + 1] && abs(l[h] - l[h + 1]) > 15)
		{
			temphl = h;
			break;
		}
		
	}
	if (temphl)
	{
		for(h = temphl + 10; h > temphl - 60 && h > 5; h--)
		{
			if(r[h] < r[h + 1] && r[h + 1] - r[h] > 10)
			{
				temphl = 0;
				break;
			}
		}
	}
	if (temphl)
	{
		float k = 0, b = 0;
		k = ((float)(l[image_h - 1] - l[temphl]) / (float)(image_h - 1 - temphl));
		b = (float)(l[image_h - 1] - k * (image_h - 1));
		for (unsigned char i = temphl - 3; i < image_h - 1; i++)
		{
			l[i] = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
		}
		
		tft180_show_string(0, 50, "thl");
	}
	
	uint8 temphr = 0;
	for (h = image_h - 9; h > 20; h--)
	{
		//如果右线向右前方, 而且差值大于15
		if (r[h] < r[h + 1] && abs(r[h] - r[h + 1]) > 15)
		{
			temphr = h;
			break;
		}
	}
	if (temphr)
	{
		for(h = temphr + 10; h > temphr - 60; h--)
		{
			if(l[h] > l[h + 1] && l[h] - l[h + 1] > 10)
			{
				temphr = 0;
				break;
			}
			
		}
	}
	if (temphr)
	{
		float k = 0, b = 0;
		k = ((float)(r[image_h - 1] - r[temphr]) / (float)(image_h - 1 - temphr));
		b = (float)(r[image_h - 1] - k * (image_h - 1));
		for (unsigned char i = temphr - 3; i < image_h - 1; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
		tft180_show_string(0, 50, "thr");
	}

}


uint8 left_down_lost;		//左下丢线标志
uint8 left_up_lost;			//左上丢线标志
uint8 left_down_y;			//左下丢线点y坐标
uint8 left_up_y;			//左上丢线点y坐标

uint8 right_down_lost;		//右下丢线标志
uint8 right_up_lost;		//右上丢线标志
uint8 right_down_y;			//右下丢线点y坐标
uint8 right_up_y;			//右上丢线点y坐标

//辅助斜入十字判断
uint8 left_max_cnt;			//左边界开始时在最左端点个数
uint8 right_max_cnt;		//右边界开始时在最右端点个数
uint8 down_reach_left;		//左下角是否到达右边界
uint8 down_reach_right;		//右下角是否到达左边界

/**
 * @brief 从图像下方向上方判断丢线(通过八邻域生长方向判断)
 * @param uint8(*image)[image_w]		输入二值图像
 * @param uint8 *l_border			输入左边界首地址
 * @param uint8 *r_border			输入右边界首地址
 * @param uint16 total_num_l			输入左边循环总次数
 * @param uint16 total_num_r			输入右边循环总次数
 * @param uint16(*points_l)[2]		输入左边轮廓首地址
 * @param uint16(*points_r)[2]		输入右边轮廓首地址
 * @see lost_line(bin_image,l_border, r_border, data_stastics_l, data_stastics_r, points_l, points_r);
 * @return void
 */
AT_ITCM_SECTION_INIT(void lost_line(uint8 *l_border, uint8 *r_border, 
									uint16 total_num_l, uint16 total_num_r,
									uint16(*points_l)[2], uint16(*points_r)[2]))
{
	uint16 i;
	left_down_lost = 0;
	left_up_lost = 0;
	left_down_y = 0;
	left_up_y = 0;
	left_max_cnt = 0;
	down_reach_left = 0;

	right_down_lost = 0;
	right_up_lost = 0;
	right_down_y = 0;
	right_up_y = 0;
	right_max_cnt = 0;
	down_reach_right = 0;
	// 左下角丢线检测 如果偏移过大则视作丢线
	if(l_border[image_h - 3] > border_min + 1 && l_border[image_h - 1] > border_min + 1 && l_border[image_h - 2] > border_min + 1)
		for(i = image_h - 1;i > image_h - 80;i--)
		{
			if(abs(l_border[i]   - l_border[i-1]) < 5 && (l_border[i] - l_border[i+1]) < 5
			&& (l_border[i-1] - l_border[i-2]) > 3 && l_border[i-1] < border_max - 5)
			{
				left_down_lost = 1;
				left_down_y = i + 1; // 记录左下丢线点y坐标
				tft180_show_string(0, 10, "l_d_l");
				break;
			}
		}
	else
		for(i = image_h - 1; i > image_h - 20; i--)
		{
			if(l_border[i] < border_min + 2)
			{
				left_max_cnt++;
			}
			if(r_border[i] > border_max - 2)
			{
				right_max_cnt++;
			}
			if(left_max_cnt >= 10)
			{
				down_reach_left = 1;
				break;
			}
			if(right_max_cnt >= 10)
			{
				down_reach_right = 1;
				break;
			}
		}
	
	// 右下角丢线检测 如果偏移过大则视作丢线
	if(r_border[image_h - 3] < border_max - 1 && r_border[image_h - 1] < border_max - 1 && r_border[image_h - 2] < border_max - 1)
		for(i = image_h - 1;i > image_h - 80;i--)
		{
			if(abs(r_border[i]   - r_border[i-1]) < 5 && (r_border[i] - r_border[i+1]) < 5
			&& (r_border[i-1] - r_border[i-2]) > 3 && r_border[i-1] > border_min + 5)
			{
				right_down_lost = 1;
				right_down_y = i + 1; // 记录右下丢线点y坐标
				tft180_show_string(0, 25, "r_d_l");
				break;
			}
		}
	// 左上角丢线检测 如果偏移过大则视作丢线 且y坐标大于左下角丢线点y坐标
	for(i = image_h - 1;i > image_h - 80;i--)
	{
		if(    l_border[i]   - l_border[i-1]  <-3 && (l_border[i] - l_border[i+1]) < 3 
		&& (l_border[i-1] - l_border[i-2]) > 3 && l_border[i-1] < border_max - 5 && i > left_down_y)
		{
			left_up_lost = 1;
			left_up_y = i + 1; // 记录左上丢线点y坐标
			tft180_show_string(0, 40, "l_u_l");
		}
	}
	// 右上角丢线检测 如果偏移过大则视作丢线 且y坐标大于右下角丢线点y坐标
	for(i = image_h - 1;i > image_h - 80;i--)
	{
		if(	  r_border[i]   - r_border[i-1]  > 3 && (r_border[i] - r_border[i+1]) < 3 
		&& (r_border[i-1] - r_border[i-2]) > 3 && r_border[i-1] > border_min + 5 && i > right_down_y)
		{
			right_up_lost = 1;
			right_up_y = i + 1; // 记录右上丢线点y坐标
			tft180_show_string(0, 55, "r_u_l");
		}
	}


/*
	uint16 jump = 3;
	float lost_slope = 2.0f;
	float prev_slope = 0, current_slope;
	for (i = 2; i < total_num_l - jump; i++)
	{
		current_slope = Slope_Calculate_xychange(i, i+jump, l_border);
		if(i == 2) prev_slope = current_slope;
		if(prev_slope - current_slope > lost_slope) left_lost = 1;
		prev_slope = current_slope;
	}
	for (i = 2; i < total_num_r - jump; i++)
	{
		current_slope = Slope_Calculate_xychange(i, i+jump, r_border);
		if(i == 2) prev_slope = current_slope;
		if(prev_slope - current_slope < lost_slope) right_lost = 1;
		prev_slope = current_slope;
	}
*/


}

/*
*******0********************************0**
**********0********************0*************
***********0**********0**********************
***************0*****************************
***************^*****************************
***************|*****************************
************这是v点**************************
*/

//===================== 并不能用 =========================
/**
 * @brief 寻找一条线的v点
 * @param uint8(*image)[image_w]	输入二值图像
 * @param total_num					总的点数
 * @param uint16(*points)[2]		单边轮廓首地址
 * @see find_v_point(image, data_statics_l, points_l)
 * @return 返回v点的y坐标
 */
AT_ITCM_SECTION_INIT(uint16 find_v_point(uint8(*image)[image_w], uint16 total_num, uint16(*points)[2]))
{
	for(uint8 i = total_num - 5; i > 5; i--)
	{
		float ave_l = arr_average((uint16*)points, i - 5, i - 1);
		float ave_r = arr_average((uint16*)points, i + 1, i + 5);
		if(points[i][1] > ave_l && points[i][1] > ave_r)
		{
			return points[i][1];
		}
	}
	return 0;

}
//===================== 并不能用 =========================
