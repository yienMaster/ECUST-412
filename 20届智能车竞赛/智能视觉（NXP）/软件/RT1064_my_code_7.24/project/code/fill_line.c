#include "zf_common_headfile.h"
#include "slope_intercept.h"
#include "image.h"
#include "lost_line.h"


/** 
 * @brief 十字补线函数
 * @param uint8(*image)[image_w]		输入二值图像
 * @param uint8 *l_border			输入左边界首地址
 * @param uint8 *r_border			输入右边界首地址
 *  @see	cross_fill(image,l_border, r_border);
 * @return void
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint16 total_num_l, uint16 total_num_r, uint8 *r_border, uint8 state)
{
	
	uint16 i;
	uint16 start, end;
	float slope_l_rate = 0, intercept_l = 0;
	float slope_r_rate = 0, intercept_r = 0;

	
	//下方补线
	if(state == 1)
	{
		//左线处理
		if(left_up_lost == 1)
		{
			slope_l_rate = (float)(l_border[left_up_y - 1] - l_border[left_down_y - 1]) / (left_up_y - left_down_y);
			intercept_l = l_border[left_down_y - 1] - slope_l_rate * (left_down_y - 1);
			//连接两拐点
			for (i = left_down_y - 1; i <= left_up_y - 1; i++)
			{
				l_border[i] = slope_l_rate * i + intercept_l;//y = kx+b
				l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
			}
		}
		else
		{
			//使用left_down_y - 5 到 left_down_y进行拟合
			start = left_down_y - 5;
			start = limit_a_b(start, 0, left_down_y - 2);
			end = left_down_y - 1;
			calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
			for (i = left_down_y - 1; i < total_num_l; i++)
			{
				l_border[i] = slope_l_rate * i + intercept_l;
				l_border[i] = limit_a_b(l_border[i], border_min, border_max);
			}
		}
		//右线处理
		if(right_up_lost == 1)
		{
			slope_r_rate = (float)(r_border[right_up_y - 1] - r_border[right_down_y - 1]) / (right_up_y - right_down_y);
			intercept_r = r_border[right_down_y - 1] - slope_r_rate * (right_down_y - 1);
			//连接两拐点
			for (i = right_down_y - 1; i <= right_up_y - 1; i++)
			{
				r_border[i] = slope_r_rate * i + intercept_r;
				r_border[i] = limit_a_b(r_border[i], border_min, border_max);
			}
		}
		else
		{
			//使用right_down_y - 5 到 right_down_y进行拟合
			start = right_down_y - 5;
			start = limit_a_b(start, 0, right_down_y - 2);
			end = right_down_y - 1;
			calculate_s_i(start, end, r_border, &slope_r_rate, &intercept_r);
			for (i = right_down_y - 1; i < total_num_r; i++)
			{
				r_border[i] = slope_r_rate * i + intercept_r;
				r_border[i] = limit_a_b(r_border[i], border_min, border_max);
			}
		}
	}
	//上方补线
	else if(state == 2)
	{
		//根据左上线拟合
		start = left_up_y;  // 从左上方拐点开始
		end = left_up_y + 5;  // 到左上方拐点+5结束
		calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
		for (i = 1; i < left_up_y; i++)
		{
			l_border[i] = slope_l_rate * (i) + intercept_l;//y = kx+b
			l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
		}

		//根据右上线拟合
		start = right_up_y;  // 从右上方拐点开始
		end = right_up_y + 5;  // 到右上方拐点+5结束
		calculate_s_i(start, end, r_border, &slope_r_rate, &intercept_r);
		for (i = 1; i <= right_up_y; i++)
		{
			r_border[i] = slope_r_rate * (i) + intercept_r;
			r_border[i] = limit_a_b(r_border[i], border_min, border_max);
		}
	}
	//右侧斜入十字
	else if(state == 3)
	{
		//连接两拐点
		slope_l_rate = (float)(l_border[left_up_y - 1] - l_border[left_down_y - 1]) / (left_up_y - left_down_y);
		intercept_l = l_border[left_down_y - 1] - slope_l_rate * (left_down_y - 1);
		
		for (i = left_down_y - 1; i <= left_up_y - 1; i++)
		{
			l_border[i] = slope_l_rate * i + intercept_l;//y = kx+b
			l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
		}
	}
	//左侧斜入十字
	else if(state == 4)
	{
		//连接两拐点
		slope_r_rate = (float)(r_border[right_up_y - 1] - r_border[right_down_y - 1]) / (right_up_y - right_down_y);
		intercept_r = r_border[right_down_y - 1] - slope_r_rate * (right_down_y - 1);
		
		for (i = right_down_y - 1; i <= right_up_y - 1; i++)
		{
			r_border[i] = slope_r_rate * i + intercept_r;//y = kx+b
			r_border[i] = limit_a_b(r_border[i], border_min, border_max);//限幅
		}
	}
		
	tft180_show_string(0,40, "c_f");

}

//左右直线判断函数
//参数：左右边线数组

/**
 * @brief	判断是否是直线
 * @param	uint8* arr	边界数组首地址
 * @return	1	是直线
 * 			0	不是直线
 */
AT_ITCM_SECTION_INIT(uint8 line_straight(uint8* arr))
{
	float slope[8];
	uint8 j = 0,flag=1;
	for (int i=image_h * 1/3 + 10;i<image_h-2;i+=10)
	{
		slope[j] = Slope_Calculate(i-10,i,arr);
		j++;
	}	
	for(int i=0;i<8;i++)
		for(j=0;j<i;j++)
			if (my_abs(slope[i] - slope[j])>0.1)
			{
				flag=0;
				break;
			}
	return flag;
}

/*
uint8 line_straight(uint8* arr)
{
	float slope_rate1,slope_rate2,slope_intercept; 
	uint8 flag=1;
	for (int i=image_h * 1/3 + 10;i<image_h-2;i+=10)
	{
		slope_rate1=calculate_s_i(i-10,i,arr);
		for (int j=image_h*1/3 + 10;j<i;j+=10)
		{
			slope_rate2=calculate_s_i(j-10,j,arr);
			if (my_abs(slope_rate2-slope_rate1)>0.1)
				flag=0;
		}
	}
	return flag;
}*/

uint8 left_flag=0;//左环岛判断
extern uint8 round_state;//环岛状态
uint8 sum1,sum2,sum3,sum4,sum5,sum6,sum7,sum8;

/**
 * @brief	环岛处理函数
 * @param void
 * @return void
 */
/*
AT_ITCM_SECTION_INIT(void road_about_dispose(void))
{
	if(left_flag)//左环岛标志位
	{
		switch(round_state)//左环岛状态
		{
			case 1:
				sum1=0;
				for (int i = 2;i<8;i++)
				{
					if(l_border[image_h-i] == 1)
						sum1 += 1;
				}
				if(sum1 >= 4) //6丢4      
				{           
					round_state = 2;//进入左环岛状态2
				}
     		break;
 
     	case 2:
				sum2 = 0;
				for (int i=2;i<8;i++)
				{
					if(l_border[image_h-i] == 1)
					sum2 += 1;
				}	
        if(sum2<4)//左下边为圆环
        {
			round_state=3;
        }
     		break;
 
     	case 3:
			if(find_v_point(bin_image, data_stastics_l, points_l)==0)//找不到v点
        	{
				round_state = 4;
        	}
     		break;
     	case 4:
			sum4 = 0;
			for (int i = 2;i < 8; i++)
			{
				if(l_border[image_h-i] == 1)
					sum4+=1;
			}
			if(right_lost == 1 && sum4 >= 4)//右边出现拐点,左边6丢4
			{
				round_state = 5;
			}
     		break;
 
     	case 5:
			if(right_lost == 0)//右边不丢线
        	{
				round_state = 6;
        	}
     		break;
 
     	case 6:
			if(find_v_point(bin_image, data_stastics_l, points_l)>0)
			{
				round_state = 7;
			}
			break;
 
     	case 7:
			sum7 = 0;
			for (int i = 2;i < 8;i++)
			{
				if(l_border[image_h-i] == 1)
					sum7 += 1;
			}
			if(sum7 < 4)
			{
				round_state = 8;
			}
			break;
 
     	case 8:
        	left_flag = 0;//清除左环岛状态；
		   	round_state = 0;//清除左环岛标志位
     		break;
 
		}
		if(round_state == 1)//左环岛状态1发现环岛
		{
			float slope_rate1,intercept1;
			calculate_s_i(break_num_l, image_h-2, l_border, &slope_rate1, &intercept1);
			for (uint8 i = break_num_l; i < image_h-2; i++)
			{
				l_border[i] = slope_rate1 * (i) + intercept1;//y= kx+b
				l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
			}
		}
		if(round_state == 2)//左环岛状态2
		{
			float slope_rate2,intercept2;
			calculate_s_i(image_h-12,image_h-2,r_border,&slope_rate2,&intercept2);
			slope_rate2=-slope_rate2;
			intercept2=l_border[break_num_l]-slope_rate2*break_num_l;
			for (uint8 i=break_num_l;i<image_h-1;i++)
			{
				l_border[i] = slope_rate2 * (i) + intercept2;//x= ky+b
				l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
			}
		}
		if(round_state == 3)//左环岛状态3进环岛
		{
			float slope_rate3,intercept3;
			uint8 v = find_v_point(bin_image, data_stastics_l, points_l);//v点坐标
			r_border[v] = l_border[v]; 
			calculate_s_i(v, image_h-2, r_border, &slope_rate3, &intercept3);
			for (uint8 i = v;i < image_h-1; i++)
			{
				r_border[i] = slope_rate3 * (i) + intercept3;//x= ky+b
				r_border[i] = limit_a_b(r_border[i], border_min, border_max);//限幅
			}
		}
 
		if(round_state == 4)//完全进入环岛
		{
			//pid正常弯道巡线；
		}
 
		if(round_state == 5)
		{
			float slope_rate4,intercept4;
			r_border[0] = 1;
			calculate_s_i(0, break_num_r, r_border, &slope_rate4, &intercept4);
			for (uint8 i = 0;i <= break_num_r; i++)
			{
				r_border[i] = slope_rate4 * (i) + intercept4;//x= ky+b
				r_border[i] = limit_a_b(r_border[i], border_min, border_max);//限幅
			}
		}
 
  if(round_state == 6)
  {
	//pid正常弯道巡线；
  }
 
  if(round_state == 7)
  {
	  float slope_rate7,intercept7;
	  uint8 v=find_v_point(bin_image, data_stastics_l, points_l);
	  calculate_s_i(image_h-12, image_h-2, r_border, &slope_rate7, &intercept7);
	  slope_rate7 = -slope_rate7;
	  intercept7 = l_border[v]-slope_rate7*v;
	  for (uint8 i = v;i < image_h - 1; i++)
	  {
		  l_border[i] = slope_rate7 * (i) + intercept7;//x= ky+b
		  l_border[i] = limit_a_b(l_border[i], border_min, border_max);//限幅
	  }
  }
   
 }
}

*/

/**
 * @brief	环岛判断函数
 * @param void
 * @return void
 */
/*
uint8 round_judge(void)
{
	if (left_flag==0)
	{
		if(left_lost==1 || right_lost==1)  //  进入元素识别状态
		{
			//左环岛判断
			if(left_lost==1||right_lost!=1)  // 左丢右不丢,可能是环岛或者弯道或者车库
				if(line_straight(r_border))  // 右边直线，排除弯道
				{
					left_flag=1;//给予左环岛标志；
					round_state=1;//进入左环岛状态1；
					return 1;
				}
		}
	}
	return 0;
}
*/