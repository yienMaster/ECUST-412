#include "my_code.h"
/*
void get_left(uint16 total_L)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	//初始化
	for (i = 0;i<image_h;i++)
	{
		l_border[i] = border_min;
		l_index[i] = 0;
	}
	h = image_h-1;
	//左边
	for (j = 0; j < total_L; j++)
	{
		//printf("%d,%d\n", points_l[j][1],h);
		if (points_l[j][1] == h)
		{
			l_border[h] = points_l[j][0]+1;
			l_index[h] = j;
		}
		else continue; //每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}
*/
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2022年9月25日
备    注：
example：get_right(data_stastics_r);
 */
/*
void get_right(uint16 total_R)
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_index[i] = 0;
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h-1;
	//右边
	for (j = 0; j < total_R; j++)
	{
		if (points_r[j][1] == h)
		{
			r_border[h] = points_r[j][0] - 1;
			r_index[h] = j;
		}
		else continue;//每行只取一个点，没到下一行就不记录
		h--;
		if (h == 0)break;//到最后一行退出
	}
}
*/

/**
 * @brief 初步判断是否是圆环( 保证没有圆环不进入状态1 )
 * @param unsigned char *l 左线数组首地址
 * @param unsigned char *r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short *indexl 左线映射数组
 */
bool iscircle(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w])
{
	unsigned char h, w;
	unsigned char leftbreak = 0;
	unsigned char rightbreak = 0;
	bool circlefind = 0;
	bool apl = 0;
	bool apr = 0;
	//从图像下方向上搜索
	for (h = image_h - 10; h > 30; h--)
	{
		//如果出现原本向右前的线下一个点到了左边
		//且下一个点和当前点差值大于5
		//且下一个点大于3 而且y坐标在20到image_h-5(从下往上第五个点)之间
		//且当前点和下一个点差值大于10(删除)
		if (l[h] < l[h + 1] && l[h + 1] >= l[h + 10]
		&& abs(l[h] - l[h + 1]) >= 5
		&& l[h + 1]>3 && h < image_h - 5 && h > 20 )
		{
											
			//(l[h+1], h+1)是白点			1      1     h-1
			//(l[h+1]+5, h-1)是白点             1        h+1
			//(l[h+1]-5, h+2)是黑点         0      1     h+2 
			//(l[h+1]+5, h+2)是白点   		^	1        h+5
			//(l[h+1]-5, h-1)是白点			|
			//(l[h+1], h+5)是白点         这个点是黑点
			if (p[h - 1][l[h + 1] - 5] == 255 && p[h - 1][l[h + 1] + 5] == 255\
			 && p[h + 2][l[h + 1] - 5] == 0   && p[h + 2][l[h + 1] + 5] == 255\
			 && p[h + 1][l[h + 1]	 ] == 255 && p[h + 5][l[h + 1]	  ] == 255)
			{
				//tft180_show_string(90, 10, "left_break");
				leftbreak = h;
				break;
			}
			
		}
		if (r[h] > r[h + 1] && r[h + 1] < r[h + 10] 
		&& abs(r[h] - r[h + 1]) >= 5
		&& r[h + 1] < border_max - 2 && h < image_h - 5 && h > 20 )
		{		
			
			//(r[h+1], h+1)是白点			1        1			
			//(r[h+1]+5, h-1)是白点             1      	上下差3
			//(r[h+1]-5, h+2)是黑点         1	     0  <--这个点是黑点
			//(r[h+1]+5, h+2)是白点   				  
			//(r[h+1]-5, h-1)是白点			    1		  
			//(r[h+1], h+5)是白点         		 
			if (p[h - 1][r[h + 1] - 5] == 255 && p[h - 1][r[h + 1] + 5] == 255\
			 && p[h + 2][r[h + 1] - 5] == 255 && p[h + 2][r[h + 1] + 5] == 0  \
			 && p[h + 1][r[h + 1]	 ] == 255 && p[h + 5][r[h + 1]	  ] == 255)
			{
				//tft180_show_string(90, 10, "right_break");
				rightbreak = h;
				break;
			}
			
		}
	}
	if (leftbreak)
	{
		apl = 1;
		//在找到左圆环初判断的点的情况下, 判断右边是否有断点
		for (h = leftbreak + 5; h > leftbreak - 15; h--)
		{
			if (r[h] - r[h - 10] > 2)
			{
				//tft180_show_string(90, 25, "right_break");
				//printf("right break\n");
				apl = 0;//有断点则退出
				break;
			}
		}
	}
	if (rightbreak)
	{
		apr = 1;
		//在找到右圆环初判断的点的情况下, 判断左边是否有断点
		for (h = rightbreak + 5; h > rightbreak - 15; h--)
		{
			if (l[h - 10] - l[h] > 2)
			{
				//tft180_show_string(90, 25, "left_break");
				//printf("right break\n");
				apr = 0;//有断点则退出
				break;
			}
		}
	}
	if (apl)//右边无断点进入圆环状态
	{
		for (h = leftbreak - 15; h > 20; h--)
		{
			if (l[h] > l[h + 5] && l[h] > l[h - 5])
			{
				//printf("right break\n");
				circlefind = true;
				break;
			}
		}
	}
	if (apr)//左边无断点进入圆环状态
	{
		for (h = rightbreak - 15; h > 20; h--)
		{
			if (r[h] < r[h + 5] && r[h] < r[h - 5])
			{
				//printf("right break\n");
				circlefind = true;
				break;
			}
		}
	}
	return  circlefind;
}

/**
 * @brief 判断是不是左圆环
 * @param unsigned char *l 左线数组首地址
 * @param unsigned char *r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short *indexl 左线映射数组
 */
bool liscircle(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char leftbreak = 0;
	unsigned char op = 0;	//圆所在点
	bool circlefind = 0;
	bool ap = 0;			//初步判断找到圆环

	//从图像下方向上搜索
	for (h = image_h - 5; h > 30; h--)
	{
		//左边线向左拐大于5, 而且左边界线的y坐标在一直递增(这样能够比较好的排除十字)
		if (l[h + 1] > l[h] && l[h + 1] - l[h] > 5\
			&&points_l[indexl[h + 1]	][1] <= points_l[indexl[h + 1] + 2][1]\
			&&points_l[indexl[h + 1]	][1] <  points_l[indexl[h + 1] + 5][1]\
			&&points_l[indexl[h + 1] + 2][1] <= points_l[indexl[h + 1] + 5][1]
			&&points_l[indexl[h + 1] + 2][0] > border_min + 1 
		)
		{
			leftbreak = h + 3;
			ap = 1;
			break;
		}
	}
	//搜完左线搜右线
	if (leftbreak)
	{
		for (w = leftbreak + 5; w > leftbreak - 35 && w > 30; w--)
		{
			//如果右边线向右前方(不是直线赛道对应的左前方)，而且没有碰到右边界
			if (r[w - 5] > r[w] && (r[w - 5] != border_max))
			{
				ap = 0;
				break;
			}
		}
	}

	if (ap)
	{
		//如果左边线向右前方(符合直线赛道特征), 而且没有碰到左边界
		for (h = leftbreak - 15; h > 20; h--)
		{
			if (l[h] > l[h + 5] && l[h] > l[h - 5] && l[h - 5] != border_min)
			{
				op = h;				//记录断点
				circlefind = 1;		//标记找到圆环
				break;
			}
		}
	}
	//如果初步找到左圆环，则进行补线，经过圆环的第一个口
	if (circlefind)
	{
		float k = 0, b = 0;
		k = ((float)(l[leftbreak] - l[op]) / (float)(leftbreak - op));
		b = (float)(l[op] - op * k);
		for (unsigned char i = op; i < leftbreak + 2; i++)
		{
			l[i] = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}

bool LFixCircle1(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{

	unsigned char h, w;
	unsigned char leftbreak = 0;
	unsigned char op = 0;
	bool circlefind = 0;
	bool ap = 0;
	//从图像下方向上搜索
	for (h = image_h - 5; h > image_h >> 1; h--)
	{
		//7月4修改
		//左边线向右前方而且左边界线的y坐标在一直递增
		if (l[h + 1] > l[h] && abs(l[h + 1] - l[h]) > 3\
			&&points_l[indexl[h + 1]][1] >= points_l[indexl[h + 1] + 3][1]\
			&&points_l[indexl[h + 1]][1] > points_l[indexl[h + 1] + 5][1]\
			&&points_l[indexl[h + 1] + 3][1] >= points_l[indexl[h + 1] + 5][1])
		{
			leftbreak = h + 3; //找到左下角的第一个拐点 
			/*           000   <----这个点
			          00 0
			     0000   0 
    	               0
			*/
			break;
		}
	}
	//如果找到第一个拐点，则找圆环的内圈圆的拐点(圆的最右边的点)
	if (leftbreak)
	{
		for (h = leftbreak - 15; h > 5; h--)
		{
			//如果左边线的l[h]处的点在两边的点的右边(行如 > )
			if (l[h] > l[h + 5] && l[h] > l[h - 5] && l[h - 5] != border_min)
			{
				op = h;				//记录圆点
				break;
			}
		}
	}
	//如果左方断点和圆的点都存在而且不重合, 则补线经过圆环的第一个口, 进入状态1
	if (op && leftbreak && op != leftbreak)
	{
		float k = 0, b = 0;
		k = ((float)(l[leftbreak] - l[op]) / (float)(leftbreak - op));
		b = (float)(l[op] - op * k);

		for (unsigned char i = op; i < leftbreak + 2; i++)
		{
			l[i] = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}

/**
 * @brief 到了左圆环第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle1(unsigned char* l, unsigned char* r, unsigned char(*p)[image_w], unsigned short* indexl)
{
	unsigned char h, w;
	unsigned char op = 0;
	bool circlefind = 0;
	bool dp = 0;

	//第二列前十个像素均为白色(找白色色块)
	//则表示过了圆环入口的第一个拐点
	if (p[image_h - 4][2] && p[image_h - 6][2] && p[image_h - 10][2])
	{
		dp = 1;
	}

	if (dp)
	{
		for (h = image_h - 15; h > 5; h--)
		{
			//如果左边线的l[h]处的点在两边的点的右边(形如 > )
			if (l[h] > l[h + 5] && l[h] > l[h - 5] && l[h - 5] != border_min)
			{
				op = h;				//记录圆点
				break;
			}
		}
	}
	//如果找到白块和内圈圆的拐点, 则将左下角和圆环的拐点连接,进入状态2
	if (dp && op)
	{
		circlefind = 1;
		float k = 0, b = 0;
		k = ((float)(l[image_h - 1] - l[op]) / (float)(image_h - 1 - op));
		b = (float)(l[op] - op * k);

		for (unsigned char i = op; i < image_h - 1; i++)
		{
			l[i] = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}

bool LFixCircle2(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char op = 0;
	bool circlefind = 0;
	bool dp = 0;

	//如果左线x坐标递增，且没有碰到边界(圆环的内环在图像左下方)

	for (h = image_h - 15; h > 5; h--)
	{
		if (l[h] > l[h + 5] && l[h] > l[h - 5] && l[h - 5] != border_min)
		{
			op = h;
			break;
		}
	}

	if (op)
	{
		float k = 0, b = 0;
		k = ((float)(l[image_h - 1] - l[op]) / (float)(image_h - 1 - op));
		b = (float)(l[op] - op * k);
		int temp = 0;
		for (unsigned char i = op; i < image_h - 1; i++)
		{
			temp = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(temp, border_min, border_max);//限幅
		}
	}
	return  circlefind;
}

/**
 * @brief 检测是否经过圆环内环
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle2(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, temph = 0;
	unsigned char v_pt = 0;
	bool circlefind = 0;
	bool dp = 0;
	//如果左下角出现黑色色块
	int8 cnt = 0;
	for (int8 i = image_h - 3; i > image_h - 10; i--)
	{
		if(!p[i][3]) cnt+=1;
		if(cnt >= 3)
		{
			dp = 1;
			break;
		}
	}
	if (dp)
	{
		/*
		//找圆环内环后的正常递增边线
		for (h = image_h - 25; h > 5; h--)
		{
			if (l[h] > l[h + 1] && abs(l[h] - l[h + 1]) > 5)
			{
				temph = h;
				break;
			}
		}
		*/
		//如果左线离开图像边界(排除特殊情况, 因为下面判断基本带等号)
		for (h = image_h - 25; h > 5; h--)
		{
			if (l[h] < border_min + 2 && l[h - 1] > l[h] && l[h - 2] >= l[h - 1])
			{
				temph = h;
				break;
			}
		}
		
	}

	if (temph)
	{
		for (uint16 j = indexl[h]; j > 0; j--)
		{
			//左边界y先递增后递减
			if( points_l[j][1] >= points_l[j + 3][1]	//左边界y递增或相等
			 && points_l[j][1] >= points_l[j + 5][1]	//左边界y递增
			 && points_l[j][1] >= points_l[j - 3][1]	//左边界y递减或相等
			 && points_l[j][1] >= points_l[j - 5][1])	//左边界y递减或相等
			{
				v_pt = h;
				circlefind = 1;
				break;
			}
			
			
		}

	}

	//把v点和右下角连接(也不完全是, 反正能进去就行)
//	if (v_pt)
//	{
//		float k = 0, b = 0;
//		k = ((float)(r[image_h - 2] - l[v_pt]) / (float)(image_h - 2 - v_pt));
//		b = (float)(l[v_pt] - v_pt * k);

//		for (unsigned char i = 0; i < image_h - 1; i++)
//		{
//			r[i] = k * (i) + b;//y = kx+b
//			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
//		}
//	}

	return  circlefind;
}

//状态3补线
//拐点和右下角连线
bool LFixCircle3(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, temph = 0;
	unsigned char vpx = 0, vpy = 0;
	/*
	uint8 cntl = 0;
	uint8 cntr = 0;
	for (h = image_h - 3; h > image_h - 30; h--)
	{
		if(l[h] < border_min + 2)cntl+=1;
		if(r[h] > border_max - 2)cntr+=1;
		if(cntl > 25 && cntr > 25)
		{
			float k = 0, b = 0;
			k = (float)(r[image_h - 1] - (border_min + 2)) / (float)(image_h - 1 - image_h/2);
			b = (float)((border_min + 2) - (image_h/2 * k));
			for (unsigned char i = 0; i < image_h - 1; i++)
			{
				r[i] = k * (i)+b;//y = kx+b
				r[i] = limit_a_b(r[i], border_min, border_max);//限幅
			}
			return 0;
		}
	}*/
	for (h = image_h - 15; h > 5; h--)
	{
		//如果左线向右前方, 而且差值大于15
		if (l[h] > l[h + 1] && abs(l[h] - l[h + 1]) > 15)
		{
			temph = h;
			break;
		}
	}

	if (temph)
	{
		for (int i = data_stastics_l - 10; i > indexl[h + 1]; i--)
		{
			/*
			if (points_l[i][1] >= points_l[i + 3][1]\
			 && points_l[i][1] > points_l[i + 5][1]\
			 && points_l[i][1] >= points_l[i - 3][1]\
			 && points_l[i][1] >= points_l[i - 5][1]\
			 && points_l[i][0] > points_l[i - 5][0]\
			 && points_l[i][0] <= points_l[i + 5][0])
			{
				vpy = points_l[i][1];
				vpx = points_l[i][0];
				break;
			}
			*/
			if( points_l[i][1] >= points_l[i + 3][1]	//左边界y递增或相等
			 && points_l[i][1] >  points_l[i + 5][1]	//左边界y递增
			 && points_l[i][1] >= points_l[i - 3][1]	//左边界y递减或相等
			 && points_l[i][1] >= points_l[i - 5][1])	//左边界y递减或相等
			{
				vpy = points_l[i][1];
				vpx = points_l[i][0];
				break;
			}
		}
	}

	if (vpy&&vpx)
	{
		float k = 0, b = 0;
		k = (float)(r[image_h - 1] - vpx) / (float)(image_h - 1 - vpy);
		b = (float)(vpx - (vpy * k));
		for (unsigned char i = 0; i < image_h - 1; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
	}
	return  0;
}

/**
 * @brief 判断是否看到岛的第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl *右右右*线映射数组
 */
bool LIsCircle3(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, temph = 0;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	unsigned char dpx = 0, dpy = 0;
	//如果图像右下角为白色而且左下角为黑色
	if (p[image_h - 4][image_w - 3]\
	&&  p[image_h - 6][image_w - 3]\
	&&  p[image_h - 8][image_w - 3]\
	&& !p[image_h - 4][3]\
	&& !p[image_h - 4][5]\
	&& !p[image_h - 2][5])
	{
		dp = 1;
	}
	/*judge right*/
	if (dp)
	{
		for (unsigned char i = image_h - 4; i > 10; i--)
		{
			//如果右线向右拐(不是正常的向左), 而且向右偏移大于5
			/*
						  00000000
	this-->	  000000000000
			   0
				0
				 0
				  0
			*/
			if (r[i - 3] > r[i] && r[i + 3] > r[i])
			{
				temph = i;
				circlefind = 1;
				break;
			}
		}
	}
	//如果找到粗判断的右拐点
	if (temph)
	{
		//从拐点开始判断
		for (int j = indexl[temph + 1]; j < data_stastics_r; j++)
		{
			//找斜向上的v点
			/*
	this-->	   000000
				0    00
				0      00000
				 0			000
				  0
			*/
			if (points_r[j][1] >= points_r[j + 3][1]\
			&&  points_r[j][1] >  points_r[j + 5][1]\
			&&  points_r[j][1] >= points_r[j - 3][1]\
			&&  points_r[j][1] >= points_r[j - 5][1]\
			&&  points_r[j][0] <  points_r[j - 5][0]\
			&&  points_r[j][0] >  points_r[j + 5][0])
			{
				dpy = points_r[j][1];
				dpx = points_r[j][0];
				break;
			}
		}
	}

	//做拐点和图像右下角的延长线
	/*
	if (dpx&&dpy)
	{
		float k = 0, b = 0;
		k = ((float)r[image_h - 1] - dpx) / (float)(image_h - 1 - dpy);
		b = (float)(dpx - (dpy * k));

		for (unsigned char i = dpy; i < image_h - 1; i++)
		{
			float temp = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(temp, border_min, border_max);//限幅
		}
	}
	*/
	return  circlefind;
}

/**
 * @brief 是否看到了第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle4(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{

	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	//如果右下角为白色色块
	for(h = image_h - 7; h > 20; h--)
	{
		//如果右线出现拐点
		if (r[h - 5] - r[h] > 6 && r[h + 5] - r[h] > 6)
		{
			return 0;
		}
	}

	return  0;
}

/**
 * @brief 判断是否出了第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle5(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	//如果右下角为白色, 则判断出了第一个口
	uint8 cnt = 0;
	for(h = image_h - 5; h > image_h / 2; h--)
	{
		if(p[h][image_w - 3]) cnt += 1;
		if(cnt >= 40)
		{
			dp = 1;
		}
	}
	return  dp;
}
bool LIsCircle5_5(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	uint8 w;
	uint8 cnt = 0;

	for(w = image_w - 3; w > 5; w-=2)
	{
		if(p[image_h - 20][w]) cnt+=1;
		if(cnt >= 85) return 1;
	}
	return 0;
}
//出圆环的第一个口补线
bool LFixCircle6(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	/*
	for (unsigned char i = image_h - 9; i > 20; i--)
	{
		//如果右线出现拐点
		if (r[i] < r[i - 3] && r[i - 3] - r[i] > 3)
		{
			vp = i;
			circlefind = 1;
			break;
		}
	}*/

	//if (vp)
	//{
		float k = 0, b = 0;
		//将拐点和图像左上角连接
		k = ((float)(0 - image_h - 3) / (float)(40 - image_w - 3));
		b = (float)(image_h - 3 - k * (image_w - 3));

		for (unsigned char i = vp; i > 0; i--)
		{
			r[i] = k * (i)+b;//y = kx+b
			l[i] = border_min;
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}

	//}
	return  circlefind;
}


/**
 * @brief 判断是否过了圆环内环
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle6(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	int8 cnt = 0;
	//如果右下角为黑色 且左下角为白色, 则判断过了圆环的内环
	for (h = image_h - 3; h > image_h - 40; h--)
	{
		if(!p[h][3]) cnt += 1;
		if(cnt >= 3)
		{
			dp = 1;
			break;
		}
	}
	cnt = 0;
	if(dp)
	{
		for (h = image_h - 3; h > image_h - 40; h--)
		{
			if(p[h][image_w - 3]) cnt += 1;
			if(cnt >= 10)
			{
				dp = 1;
				break;
			}
		}
		dp = 0;
	}

	return  dp;
}


bool LFixCircle7(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h = 0, temph = 0;
	unsigned char vpx = 0, vpy = 0;
	bool circlefind = 0;
	bool dp = 0;

	/*left straight & right break*/
	for (h = image_h - 9; h > 0; h--)
	{
		//如果左线向右前方, 而且差值大于15
		if (l[h] > l[h + 1] && abs(l[h] - l[h + 1]) > 15)
		{
			temph = h;
			break;
		}
	}


	if (temph)
	{
		float k = 0, b = 0;
		k = ((float)(l[image_h - 1] - l[temph]) / (float)(image_h - 1 - temph));
		b = (float)(l[image_h - 1] - k * (image_h - 1));
		for (unsigned char i = temph - 3; i < image_h - 1; i++)
		{
			l[i] = k * (i)+b;//y = kx+b
			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
		}
	}

	return  circlefind;
}


/**
 * @brief 判断是否走出圆环进入正常赛道
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool LIsCircle7(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{
	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	//如果且左下角为黑色
	if (!p[image_h - 5][3] && !p[image_h - 3][3] && !p[image_h - 7][3])
	{
		dp = 1;
	}
	if(dp)
	{
		for (int8 i = image_h - 5; i > 10; i--)
		{
			// 如果左线不是正常向右前方(即不符合赛道特征)
			if(l[i + 1] - l[i] > 1 && r[i] - r[i + 1] > 1)
			{
				// 则判断未走出圆环
				dp = 0;
				break;
			}
		}
	}

	return  dp;
}


/**
 * @brief 判断是不是右圆环
 * @param unsigned char *l 左线数组首地址
 * @param unsigned char *r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short *indexl 左线映射数组
 */
bool riscircle(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{
	unsigned char h, w;
	unsigned char rightbreak = 0;
	unsigned char op = 0;	//圆所在点
	bool circlefind = 0;
	bool ap = 0;			//初步判断找到圆环

	//从图像下方向上搜索
	for (h = image_h - 5; h > 30; h--)
	{
		//右边线向右拐大于5, 而且右边界线的y坐标在一直递增(这样能够比较好的排除十字)
		if (r[h + 1] < r[h] && r[h] - r[h + 1] > 5\
			&&points_r[indexr[h + 1]	][1] <= points_r[indexr[h + 1] + 2][1]\
			&&points_r[indexr[h + 1]	][1] <  points_r[indexr[h + 1] + 5][1]\
			&&points_r[indexr[h + 1] + 2][1] <= points_r[indexr[h + 1] + 5][1])
		{
			rightbreak = h + 3;
			ap = 1;
			break;
		}
	}
	//搜完右线搜左线
	if (rightbreak)
	{
		for (w = rightbreak + 5; w > rightbreak - 25 && w > 30; w--)
		{
			//如果左边线向左前方(不是直线赛道对应的左前方)，而且没有碰到左边界
			if (l[w - 5] < l[w] && (l[w - 5] != border_min))
			{
				ap = 0;
				break;
			}
		}
	}

	if (ap)
	{
		for (h = rightbreak - 15; h > 20; h--)
		{
			//如果右线形状如 <
			if (r[h] < r[h + 5] && r[h] < r[h - 5] && r[h - 5] != border_max)
			{
				op = h;				//记录断点
				circlefind = 1;		//标记找到圆环
				break;
			}
		}
	}
	//如果初步找到右圆环，则进行补线，经过圆环的第一个口
	if (circlefind)
	{
		float k = 0, b = 0;
		k = ((float)(r[rightbreak] - r[op]) / (float)(rightbreak - op));
		b = (float)(r[op] - op * k);
		for (unsigned char i = op; i < rightbreak + 2; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}


/**
 * @brief 到了右圆环第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexr 右线映射数组
 */
bool RIsCircle1(unsigned char* l, unsigned char* r, unsigned char(*p)[image_w], unsigned short* indexr)
{
	unsigned char h, w;
	unsigned char op = 0;
	bool circlefind = 0;
	bool dp = 0;

	//倒数第二列前十个像素均为白色(找白色色块)
	//则表示过了圆环入口的第一个拐点
	if (p[image_h - 4][image_w - 3] && p[image_h - 6][image_w - 3] && p[image_h - 10][image_w - 3])
	{
		dp = 1;
	}

	if (dp)
	{
		for (h = image_h - 15; h > 5; h--)
		{
			//如果左边线的l[h]处的点在两边的点的右边(形如 < )
			if (r[h] < r[h + 5] && r[h] < r[h - 5] && r[h - 5] != border_max)
			{
				op = h;				//记录圆点
				break;
			}
		}
	}
	//如果找到白块和内圈圆的拐点, 则将左下角和圆环的拐点连接,进入状态2
	if (dp && op)
	{
		circlefind = 1;
		float k = 0, b = 0;
		k = ((float)(r[image_h - 1] - r[op]) / (float)(image_h - 1 - op));
		b = (float)(r[op] - op * k);

		for (unsigned char i = op; i < image_h - 1; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}


bool RFixCircle1(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{

	unsigned char h, w;
	unsigned char rightbreak = 0;
	unsigned char op = 0;
	bool circlefind = 0;
	bool ap = 0;
	//从图像下方向上搜索
	for (h = image_h - 5; h > image_h >> 1; h--)
	{
		//右边线向左前方而且右边界线的y坐标在一直递增
		if (r[h + 1] < r[h] && abs(r[h + 1] - r[h]) > 3\
			&&points_r[indexr[h + 1]	][1] >= points_r[indexr[h + 1] + 3][1]\
			&&points_r[indexr[h + 1]	][1] >  points_r[indexr[h + 1] + 5][1]\
			&&points_r[indexr[h + 1] + 3][1] >= points_r[indexr[h + 1] + 5][1])
		{
			rightbreak = h + 3; //找到左下角的第一个拐点 

			break;
		}
	}
	//如果找到第一个拐点，则找圆环的内圈圆的拐点(圆的最右边的点)
	if (rightbreak)
	{
		for (h = rightbreak - 15; h > 5; h--)
		{
			//如果左边线的l[h]处的点在两边的点的右边(行如 < )
			if (r[h] < r[h + 5] && r[h] < r[h - 5] && r[h - 5] != border_max)
			{
				op = h;				//记录圆点
				break;
			}
		}
	}
	//如果左方断点和圆的点都存在而且不重合, 则补线经过圆环的第一个口, 进入状态1
	if (op && rightbreak && op != rightbreak)
	{
		float k = 0, b = 0;
		k = ((float)(r[rightbreak] - r[op]) / (float)(rightbreak - op));
		b = (float)(r[op] - op * k);

		for (unsigned char i = op; i < rightbreak + 2; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
	}
	return  circlefind;
}


/**
 * @brief 检测是否经过圆环内环
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool RIsCircle2(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{
	unsigned char h, temph = 0;
	unsigned char v_pt = 0;
	bool circlefind = 0;
	bool dp = 0;
	//如果左下角出现黑色色块
	int8 cnt = 0;
	for (int8 i = image_h - 3; i > image_h - 10; i--)
	{
		if(!p[i][image_w - 3]) cnt+=1;
		if(cnt >= 3)
		{
			dp = 1;
			break;
		}
	}
	if (dp)
	{
		/*
		//找圆环内环后的正常递增边线
		for (h = image_h - 25; h > 5; h--)
		{
			if (l[h] > l[h + 1] && abs(l[h] - l[h + 1]) > 5)
			{
				temph = h;
				break;
			}
		}
		*/
		//如果左线离开图像边界(排除特殊情况, 因为下面判断基本带等号)
		for (h = image_h - 25; h > 5; h--)
		{
			if (r[h] > border_max - 2 && r[h - 1] < r[h] && r[h - 2] <= r[h - 1])
			{
				temph = h;
				break;
			}
		}
		
	}

	if (temph)
	{
		for (uint16 j = indexr[h]; j > 0; j--)
		{
			//左边界y先递增后递减
			if( points_r[j][1] >= points_r[j + 3][1]	//左边界y递增或相等
			 && points_r[j][1] >= points_r[j + 5][1]	//左边界y递增
			 && points_r[j][1] >= points_r[j - 3][1]	//左边界y递减或相等
			 && points_r[j][1] >= points_r[j - 5][1])	//左边界y递减或相等
			{
				v_pt = h;
				circlefind = 1;
				break;
			}
			
			
		}

	}

	//把v点和右下角连接(也不完全是, 反正能进去就行)
//	if (v_pt)
//	{
//		float k = 0, b = 0;
//		k = ((float)(l[image_h - 2] - r[v_pt]) / (float)(image_h - 2 - v_pt));
//		b = (float)(r[v_pt] - v_pt * k);

//		for (unsigned char i = 0; i < image_h - 1; i++)
//		{
//			l[i] = k * (i) + b;//y = kx+b
//			l[i] = limit_a_b(l[i], border_min, border_max);//限幅
//		}
//	}

	return  circlefind;
}


bool RFixCircle2(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{
	unsigned char h, w;
	unsigned char op = 0;
	bool circlefind = 0;
	bool dp = 0;

	//如果左线x坐标递增，且没有碰到边界(圆环的内环在图像左下方)

	for (h = image_h - 15; h > 5; h--)
	{
		if (r[h] < r[h + 5] && r[h] < r[h - 5] && r[h - 5] != border_max)
		{
			op = h;
			break;
		}
	}

	if (op)
	{
		float k = 0, b = 0;
		k = ((float)(r[image_h - 1] - r[op]) / (float)(image_h - 1 - op));
		b = (float)(r[op] - op * k);
		int temp = 0;
		for (unsigned char i = op; i < image_h - 1; i++)
		{
			temp = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(temp, border_min, border_max);//限幅
		}
	}
	return  circlefind;
}


/**
 * @brief 是否看到了第一个口
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool RIsCircle4(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexl)
{

	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	//如果右下角为白色色块
	for(h = image_h - 7; h > 20; h--)
	{
		//如果左线出现拐点
		if (l[h] - l[h - 5] > 6 && l[h] - l[h + 5] > 6)
		{
			return 0;
		}
	}

	return 1;
}


bool RFixCircle7(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{
	unsigned char h = 0, temph = 0;
	unsigned char vpx = 0, vpy = 0;
	bool circlefind = 0;
	bool dp = 0;

	/*left straight & right break*/
	for (h = image_h - 9; h > 0; h--)
	{
		//如果左线向右前方, 而且差值大于15
		if (r[h] < r[h + 1] && abs(r[h] - r[h + 1]) > 15)
		{
			temph = h;
			break;
		}
	}


	if (temph)
	{
		float k = 0, b = 0;
		k = ((float)(r[image_h - 1] - r[temph]) / (float)(image_h - 1 - temph));
		b = (float)(r[image_h - 1] - k * (image_h - 1));
		for (unsigned char i = temph - 3; i < image_h - 1; i++)
		{
			r[i] = k * (i)+b;//y = kx+b
			r[i] = limit_a_b(r[i], border_min, border_max);//限幅
		}
	}

	return  circlefind;
}


/**
 * @brief 判断是否走出圆环进入正常赛道
 * @param unsigned char* l 左线数组首地址
 * @param unsigned char* r 右线数组首地址
 * @param unsigned char(*p)[image_w] 二值化的图像
 * @param unsigned short* indexl 左线映射数组
 */
bool RIsCircle7(unsigned char *l, unsigned char *r, unsigned char(*p)[image_w], unsigned short *indexr)
{
	unsigned char h, w;
	unsigned char vp = 0;
	bool circlefind = 0;
	bool dp = 0;
	//如果且右下角为黑色
	if (!p[image_h - 5][image_w - 3] && !p[image_h - 3][image_w - 3] && !p[image_h - 7][image_w - 3])
	{
		dp = 1;
	}
	if(dp)
	{
		for (int8 i = image_h - 5; i > 10; i--)
		{
			// 如果左线不是正常向右前方(即不符合赛道特征)
			if(r[i + 1] - l[i] > 1 && r[i] - r[i + 1] > 1)
			{
				// 则判断未走出圆环
				dp = 0;
				break;
			}
		}
	}

	return  dp;
}


int32 time_schedule_01s = 0;
char circle_state[20];
void circletemp(unsigned char(*img)[image_w], unsigned char *l, unsigned char *r)
{
	static unsigned char statues = 0, circletype = 0;//左圆环:1 右圆环:2
	//sprintf(circle_state, "%d", statues);
	//tft180_show_string(0, 10, circle_state);
	bool lflag = 0, rflag = 0;
	//判断是否找到圆环
	if (statues == 0)
	{
		//if(iscircle(l, r, img))
		//{
		lflag = liscircle(l, r, img, l_index);
		rflag = riscircle(l, r, img, r_index);
		if (lflag || rflag)
		{
			//gpio_set_level(BUZZER_PIN, 1);
			statues = 1;
			if (lflag)circletype = 1;
			if (rflag)circletype = 2;
		}
		//}
		time_schedule_01s = 0;
	}
	
	// ============== 左圆环补线 ==============
	if(circletype == 1)
	{
		switch (statues)
		{
			case 1:
				// 刚找到左圆环
				if(LIsCircle1(l, r, img, l_index))statues = 2;
				LFixCircle1(l, r, img, l_index);
				//gpio_set_level(BUZZER_PIN, 1);
				time_schedule_01s = 0;
				break;
			case 2:
				// 到了左圆环第一个口
				if(LIsCircle2(l, r, img, l_index))statues = 3;
				LFixCircle2(l, r, img, l_index);
				if(!time_schedule_01s) time_schedule_01s = cnt_01s;
				break;
			case 3:
				if(cnt_01s - time_schedule_01s >= 2)
				{
					statues = 4;
					time_schedule_01s = cnt_01s;
				}
				//if (LIsCircle3(l, r, img, r_index))statues = 4;
				//LFixCircle3(l, r, img, l_index);
				
				break;
			case 4:
				if (LIsCircle4(l, r, img, l_index) || (cnt_01s - time_schedule_01s >= 4))
				{
					statues = 5;
					time_schedule_01s = cnt_01s;
					
				}
				gpio_set_level(BUZZER_PIN, 0);
				LFixCircle7(l, r, img, l_index);

				break;
			case 5:
				// 判断是否走出圆环进入正常赛道
				if (LIsCircle7(l, r, img, l_index) || (cnt_01s - time_schedule_01s >= 4))
				{
					statues = 0;
					gpio_set_level(BUZZER_PIN, 0);
					ort_label[ort_line] = 25;
					ort_label[ort_line + 1] = 25;
					ort_line+=2;
				}
				LFixCircle7(l, r, img, l_index);
				
				break;
			default:
				break;
		}
	}
	if(circletype == 2)
	{
		switch(statues)
		{
			
			case 1:
				if(RIsCircle1(l, r, img, r_index)) statues = 2;
				RFixCircle1(l, r, img, r_index);
				//gpio_set_level(BUZZER_PIN, 1);
				time_schedule_01s = 0;
				break;
			case 2:
				if(RIsCircle2(l, r, img, r_index)) statues = 3;
				RFixCircle2(l, r, img, r_index);
				if(!time_schedule_01s) time_schedule_01s = cnt_01s;
				break;
			case 3:
				if(cnt_01s - time_schedule_01s >= 2)
				{
					statues = 4;
					time_schedule_01s = cnt_01s;
				}
				break;
			case 4:
				if (RIsCircle4(l, r, img, r_index) || (cnt_01s - time_schedule_01s >= 4))
				{
					statues = 5;
					time_schedule_01s = cnt_01s;
					
				}
				gpio_set_level(BUZZER_PIN, 0);
				RFixCircle7(l, r, img, r_index);
			case 5:
				// 判断是否走出圆环进入正常赛道
				if (RIsCircle7(l, r, img, r_index) || (cnt_01s - time_schedule_01s >= 4))
				{
					statues = 0;
					gpio_set_level(BUZZER_PIN, 0);
					ort_label[ort_line] = 25;
					ort_label[ort_line + 1] = 25;
					ort_line+=2;
				}
				RFixCircle7(l, r, img, r_index);

		}
	}
}
/*
void circletemp(unsigned char(*img)[image_w], unsigned char *l, unsigned char *r)
{
	static unsigned char statues = 0, circletype = 0;//左圆环:1 右圆环:2
	sprintf(circle_state, "%d", statues);
	tft180_show_string(0, 10, circle_state);
	bool lflag = 0, rflag = 0;
	//判断是否找到圆环
	if (statues == 0)
	{
		//if(iscircle(l, r, img))
		//{
			lflag = liscircle(l, r, img, l_index);
			rflag = riscircle2(l, r, img, r_index);
			if (lflag || rflag)
			{
				//gpio_set_level(BUZZER_PIN, 1);
				statues = 1;
				if (lflag)circletype = 1;
				if (rflag)circletype = 2;
			}
		//}
		time_schedule_01s = 0;
	}
	
	// ============== 左圆环补线 ==============
	if(circletype == 1)
	{
		switch (statues)
		{
			case 1:
				// 刚找到左圆环
				if(LIsCircle1(l, r, img, l_index))statues = 2;
				LFixCircle1(l, r, img, l_index);
				gpio_set_level(BUZZER_PIN, 1);
				break;
			case 2:
				// 到了左圆环第一个口
				if(LIsCircle2(l, r, img, l_index))statues = 3;
				LFixCircle2(l, r, img, l_index);
				if(!time_schedule_01s) time_schedule_01s = cnt_01s;
				break;
			case 3:
				// 检测是否经过圆环内环
				if(cnt_01s - time_schedule_01s >= 2)
				{
					statues = 4;
					time_schedule_01s = cnt_01s;
				}
				//if (LIsCircle3(l, r, img, r_index))statues = 4;
				//LFixCircle3(l, r, img, l_index);
				
				break;
			case 4:
				// 是否看到第一个口, 并且等待1.5s
				if (LIsCircle4(l, r, img, l_index) && cnt_01s - time_schedule_01s >= 15)
				{
					statues = 5;
					time_schedule_01s = cnt_01s;
					
				}
				gpio_set_level(BUZZER_PIN, 0);
				LFixCircle3(l, r, img, l_index);

				break;
			case 5:
				// 是否出了第一个口
				if (LIsCircle5(l, r, img, l_index) && cnt_01s - time_schedule_01s >= 10)statues = 6;
				LFixCircle6(l, r, img, r_index);
				
				break;
			case 6:
				// 判断是否过了圆环内环
				
				if (LIsCircle6(l, r, img, l_index))statues = 7;
				if(LIsCircle5_5(l, r, img, l_index)) LFixCircle6(l, r, img, r_index);
				else LFixCircle7(l, r, img, r_index);

				break;
			case 7:
				// 判断是否走出圆环进入正常赛道
				if (LIsCircle7(l, r, img, l_index))
				{
					statues = 0;
					gpio_set_level(BUZZER_PIN, 0);
				}
				LFixCircle7(l, r, img, r_index);
				
				break;
			default:
				break;
		}
	}
	*/
	// ============== 右圆环补线 ==============

	/*
	
	// 刚找到左圆环
	if (statues == 1 && circletype == 1)
	{
		//gpio_set_level(BUZZER_PIN, 1);
		bool is1find = 0;
		is1find = LIsCircle1(l, r, img, l_index);
		LFixCircle1(l, r, img, l_index);
		if (is1find)statues = 2;
	}
	// 到了左圆环第一个口
	if (statues == 2 && circletype == 1)
	{
		bool is2find = 0;
		is2find = LIsCircle2(l, r, img, l_index);
		LFixCircle2(l, r, img, l_index);
		if (is2find)statues = 3;
	}
	// 检测是否经过圆环内环
	if (statues == 3 && circletype == 1)
	{
		bool is3find = 0;
		is3find = LIsCircle3(l, r, img, r_index);
		LFixCircle3(l, r, img, l_index);
		if (is3find)statues = 4;
	}
	// 是否出了第一个口
	if (statues == 4 && circletype == 1)
	{
		bool is4find = 0;
		is4find = LIsCircle4(l, r, img, l_index);
		//LFixCircle5(l, r, img, l_index);
		if (is4find)statues = 5;
	}
	
	// 判断是否过了圆环内环
	if (statues == 5 && circletype == 1)
	{
		bool is5find = 0;
		is5find = LIsCircle5(l, r, img, l_index);
		LFixCircle7(l, r, img, r_index);
		if (is5find)statues = 6;
	}

	// 判断是否走出圆环进入正常赛道
	if (statues == 6 && circletype == 1)
	{
		bool is6find = 0;
		is6find = LIsCircle6(l, r, img, l_index);
		LFixCircle7(l, r, img, r_index);
		if (is6find)statues = 7;
	}
	if (statues == 7 && circletype == 1)
	{
		bool is7find = 0;
		is7find = LIsCircle7(l, r, img, l_index);
		LFixCircle7(l, r, img, r_index);
		if (is7find)statues = 0;
	}*/

	
	

//}


