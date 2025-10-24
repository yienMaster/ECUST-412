#include "my_code.h"
#include "math.h"


/**
 * @brief 求绝对值
 * @param int value 输入值
 * @return value的绝对值
 */
AT_ITCM_SECTION_INIT(int my_abs(int value))
{
	if(value >= 0) return value;
	else return -value;
}

/**
 * @brief 限制数字大小
 * @param int16 x    需要限制的数字
 * @param int min    下限
 * @param int max	 	 上限
 * @return 限制后的数字x
*/
AT_ITCM_SECTION_INIT(int limit_a_b(int x, int min, int max))
{
    if(x<min) x = min;
    if(x>max) x = max;
    return x;
}

uint8 original_image[image_h][image_w];
uint8 image_thereshold;//图像分割阈值

/**
 * @brief 获得一副灰度图像
 * @param void
 * @return void
 */
AT_ITCM_SECTION_INIT(void Get_image(uint8(*mt9v03x_image)[image_w]))
{
#define use_num		1	//1就是不压缩，2就是压缩一倍	
	uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//这里的参数填写你的摄像头采集到的图像
						line++;
        }
        line = 0;
        row++;
    }
}

/**
 * @brief 大津法求动态阈值
 * @param uint8* image	传入图像
 * @param uint16 col	图像宽度
 * @param uint16 row	图像高度
 * @return threshold	目标阈值
 */
AT_ITCM_SECTION_INIT(uint8 otsuThreshold(uint8* image, uint16 col, uint16 row))
{
#define GrayScale 256
    uint16 Image_Width  = col;
    uint16 Image_Height = row;
    int X; uint16 Y;
    uint8* data = image;
    int HistGram[GrayScale] = {0};
	
	uint32 Amount = 0;
    uint32 PixelBack = 0;
    uint32 PixelIntegralBack = 0;
    uint32 PixelIntegral = 0;
    int32 PixelIntegralFore = 0;
    int32 PixelFore = 0;
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // 类间方差;
    int MinValue=0, MaxValue=0;
    uint8 Threshold = 0;
	
	
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height改为Y =Image_Height；以便进行 行二值化
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        	HistGram[(int)data[Y*Image_Width + X]]++; //统计每个灰度值的个数信息
        }
    }


    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //获取最小灰度的值
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //获取最大灰度的值

    if (MaxValue == MinValue)
    {
        return MaxValue;          // 图像中只有一个颜色
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // 图像中只有二个颜色
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  像素总数
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//灰度值总数
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //前景像素点数
          PixelFore = Amount - PixelBack;         //背景像素点数
          OmegaBack = (double)PixelBack / Amount;//前景像素百分比
          OmegaFore = (double)PixelFore / Amount;//背景像素百分比
          PixelIntegralBack += HistGram[Y] * Y;  //前景灰度值
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//背景灰度值
          MicroBack = (double)PixelIntegralBack / PixelBack;//前景灰度百分比
          MicroFore = (double)PixelIntegralFore / PixelFore;//背景灰度百分比
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//遍历最大的类间方差g
          {
              SigmaB = Sigma;
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
uint8 bin_image[image_h][image_w];//图像数组

/**
 * @brief 图像二值化,存储到bin_image数组里
 * @param void
 * @return void
 */
AT_ITCM_SECTION_INIT(void turn_to_bin(void))
{
	uint8 i,j;
	image_thereshold = otsuThreshold(original_image[0], image_w, image_h);
	for(i = 0;i<image_h;i++)
	{
		for(j = 0;j<image_w;j++)
      	{
    		if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
        	else bin_image[i][j] = black_pixel;
    	}
	}
}


 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线 第二个下标0是x,1是y
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点	
	
//定义八个邻域
AT_DTCM_SECTION_ALIGN_INIT(int8 seeds_l[8][2], 8) = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是顺时针
AT_DTCM_SECTION_ALIGN_INIT(int8 seeds_r[8][2], 8) = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
	//{-1,-1},{0,-1},{+1,-1},
	//{-1, 0},	     {+1, 0},
	//{-1,+1},{0,+1},{+1,+1},
	//这个是逆时针
/**
 * @brief 八邻域搜左右线
 * @param uint16 break_flag 			最多的搜索次数
 * @param uint8 (*image)[image_w]		需要进行找点的图像数组，必须是二值图,填入数组名称即可
 * @param uint16* l_stastic				统计左边数据，用来输入初始数组成员的序号和取出循环次数
 * @param uint16* r_stastic				统计右边数据，用来输入初始数组成员的序号和取出循环次数
 * @param uint16 l_start_x				左边起点横坐标
 * @param uint16 r_start_x				右边起点横坐标
 * @param uint8* hightest				循环结束所得到的最高高度
 *  @see  		search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,
 * 							start_point_l[0],start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 * @return void
 */
AT_ITCM_SECTION_INIT(void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic,
										 uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8* hightest))
{

	
	uint8 i = 0, j = 0;

	//左边变量
	uint8 search_filds_l[8][2] = { {  0 } };
	uint8 index_l = 0;
	uint8 temp_l[8][2] = { {  0 } };
	uint8 center_point_l[2] = {  0 };
	uint16 l_data_statics;//统计左边
	

	//右边变量
	uint8 search_filds_r[8][2] = { {  0 } };
	uint8 center_point_r[2] = { 0 };//中心坐标点
	uint8 index_r = 0;//索引下标
	uint8 temp_r[8][2] = { {  0 } };
	uint16 r_data_statics;//统计右边
	

	l_data_statics = *l_stastic;//统计找到了多少个点，方便后续把点全部画出来
	r_data_statics = *r_stastic;//统计找到了多少个点，方便后续把点全部画出来

	//第一次更新坐标点  将找到的起点值传进来
	center_point_l[0] = l_start_x;//x
	center_point_l[1] = l_start_y;//y
	center_point_r[0] = r_start_x;//x
	center_point_r[1] = r_start_y;//y

		//开启邻域循环
	while (break_flag--)
	{
		if(r_start_x - l_start_x < 3) break;
		//左边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			//左边使用逆时针
			search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
			search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_l[l_data_statics][0] = center_point_l[0];//x
		points_l[l_data_statics][1] = center_point_l[1];//y
		l_data_statics++;//索引加一

		//右边
		for (i = 0; i < 8; i++)//传递8F坐标
		{
			//右边使用顺时针
			search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
			search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
		}
		//中心坐标点填充到已经找到的点内
		points_r[r_data_statics][0] = center_point_r[0];//x
		points_r[r_data_statics][1] = center_point_r[1];//y

		index_l = 0;//先清零，后使用
		for (i = 0; i < 8; i++)
		{
			temp_l[i][0] = 0;//先清零，后使用
			temp_l[i][1] = 0;//先清零，后使用
		}

		//左边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
				&& image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
			{
				temp_l[index_l][0] = search_filds_l[(i)][0];
				temp_l[index_l][1] = search_filds_l[(i)][1];
				index_l++;
				dir_l[l_data_statics - 1] = (i);//记录生长方向
			}

			if (index_l)
			{
				//更新坐标点
				center_point_l[0] = temp_l[0][0];//x
				center_point_l[1] = temp_l[0][1];//y
				for (j = 0; j < index_l; j++)
				{
					if (center_point_l[1] > temp_l[j][1])
					{
						center_point_l[0] = temp_l[j][0];//x
						center_point_l[1] = temp_l[j][1];//y
					}
				}
			}

		}
		int8 dir_prev, dir_curr, delta;
		if(l_data_statics >= 5)
		{
			dir_prev = dir_l[l_data_statics-2];
			dir_curr = dir_l[l_data_statics-1];
			delta = abs(dir_curr - dir_prev);
			if (delta == 4)
			{
					//tft180_show_string(0, 20, "dl");
					break;
			}
		}
		if(r_data_statics >= 5)
		{
			dir_prev = dir_r[r_data_statics-2];
			dir_curr = dir_r[r_data_statics-1];
			delta = abs(dir_curr - dir_prev);
			if (delta == 4)
			{
					//tft180_show_string(0, 50, "dr");
					break;
			}
		}
//		uint8 dupe_point_l = dir_l[l_data_statics-1] + dir_l[l_data_statics];
//		uint8 dupe_point_r = dir_r[r_data_statics-1] + dir_r[r_data_statics];

//		if(dupe_point_l > 3 && dupe_point_l < 11 && dupe_point_l % 2 == 0)
//		{
//			tft180_show_string(0, 90, "dupe_l_p");
//			//uart_write_string(UART1_INDEX,"dupe left point\n");
//			break;//两个点之间互相进入导致卡死则退出
//		}
//		if(dupe_point_r > 3 && dupe_point_r < 11 && dupe_point_r % 2 == 0)
//		{
//			tft180_show_string(0, 100, "dupe_r_p");
//			//uart_write_string(UART1_INDEX,"dupe right point\n");
//			break;//两个点之间互相进入导致卡死则退出
//		}
		if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
			&& points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
			||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
				&& points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
		{
			//tft180_show_string(0, 80, "d3t");
			//uart_write_string(UART1_INDEX,"dupe 3 times\n");
			break;
		}
		if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
			&& my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
			)
		{
			//tft180_show_string(0, 110, "lmr");
			//uart_write_string(UART1_INDEX,"left meet right\n");
			*hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
			//printf("\n在y=%d处退出\n",*hightest);
			break;
		}
		
		if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
		{
			//uart_write_string(UART1_INDEX,"left higher than right\n");
			continue;//如果左边比右边高了，左边等待右边
		}
		if (dir_l[l_data_statics - 1] == 7
			&& (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
		{
			//uart_write_string(UART1_INDEX,"left down waiting right\n");
			center_point_l[0] = points_l[l_data_statics - 1][0];//x
			center_point_l[1] = points_l[l_data_statics - 1][1];//y
			l_data_statics--;
		}
		r_data_statics++;//索引加一

		index_r = 0;//先清零后使用
		for (i = 0; i < 8; i++)
		{
			temp_r[i][0] = 0;//先清零后使用
			temp_r[i][1] = 0;//先清零后使用
		}

		//右边判断
		for (i = 0; i < 8; i++)
		{
			if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
				&& image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
			{
				temp_r[index_r][0] = search_filds_r[(i)][0];
				temp_r[index_r][1] = search_filds_r[(i)][1];
				index_r++;//索引加一
				dir_r[r_data_statics - 1] = (i);//记录生长方向
				//printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
			}
			if (index_r)
			{

				//更新坐标点
				center_point_r[0] = temp_r[0][0];//x
				center_point_r[1] = temp_r[0][1];//y
				for (j = 0; j < index_r; j++)
				{
					if (center_point_r[1] > temp_r[j][1])
					{
						center_point_r[0] = temp_r[j][0];//x
						center_point_r[1] = temp_r[j][1];//y
					}
				}

			}
		}

	}

	//取出循环次数
	*l_stastic = l_data_statics;
	*r_stastic = r_data_statics;

}

uint8 l_border[image_h];//左线数组(从上往下 (y小到y大))
uint8 r_border[image_h];//右线数组(从上往下 (y小到y大))
uint8 center_line[image_h];//中线数组

uint16 l_index[image_h];
uint16 r_index[image_h];//映射关系
/**
 * @brief 从八邻域边界里提取需要的边线（ 0索引是最下方的点(y最大) )
 * @param uint16 total_L	找到的点的总数
 *  @see get_left(data_stastics_l );
 * @return void
*/
AT_ITCM_SECTION_INIT(void get_left(uint16 total_L))
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
	h = image_h-2;
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

/**
 * @brief 从八邻域边界里提取需要的右边线
 * @param uint16 total_R	找到的点的总数
 *  @see get_right(data_stastics_r );
 * @return void
*/
AT_ITCM_SECTION_INIT(void get_right(uint16 total_R))
{
	uint8 i = 0;
	uint16 j = 0;
	uint8 h = 0;
	for (i = 0; i < image_h; i++)
	{
		r_index[i] = 0;
		r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
	}
	h = image_h-2;
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

//定义膨胀和腐蚀的阈值区间
#define threshold_max	255 * 5 // 3*3的宫格中 6个白点就膨胀中心
/*	1:白 0:黑
110		110
101 -> 	111
010		010
*/
#define threshold_min	255 * 2 // 3*3的宫格中 3个白点就腐蚀中心
/*	1:白 0:黑
000		000
110 ->	100
001		001
*/

/**
 * @brief 形态学滤波
 * @param uint8(*bin_image)[image_w]	传入的二值化图像
 * @return void
 */
AT_ITCM_SECTION_INIT(void image_not_so_much_filter(uint8(*bin_image)[image_w]))//形态学滤波，简单来说就是膨胀和腐蚀的思想
{
	uint16 i, j;
	uint32 num = 0;


	for (i = 1; i < image_h - 1; i++)
	{
		for (j = 1; j < (image_w - 1); j++)
		{
			//统计八个方向的像素值
			num =
				bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
				+ bin_image[i][j - 1] + bin_image[i][j + 1]
				+ bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


			if (num >= threshold_max && bin_image[i][j] == 0)
			{
				bin_image[i][j] = white_pixel;//白
			}
			if (num <= threshold_min && bin_image[i][j] == 255)
			{
				bin_image[i][j] = black_pixel;//黑
			}

		}
	}
	
}
uint8 map_image[image_h][image_w] = {0};
//d=21；sigmaColor=10；sigmaSpace = 10
void bilateralFilter(uint8(*img)[image_w], int d, double sigmaColor, double sigmaSpace) {
  // 假设 map_image 已正确声明并分配内存，尺寸为 image_h x image_w
	int half_d = d / 2; // 计算窗口半径
	for (int i = 0; i < image_h; i++) {        // 遍历所有行
		for (int j = 0; j < image_w; j++) {    // 遍历所有列
		double weightSum = 0.0;
		double filterValue = 0.0;

		// 遍历以 (i,j) 为中心的 d x d 邻域
		for (int row_d = -half_d; row_d <= half_d; row_d++) {
			for (int col_d = -half_d; col_d <= half_d; col_d++) {
				// 计算邻域像素坐标
				int neighbor_x = i + row_d;
				int neighbor_y = j + col_d;

				// 跳过越界的邻域像素
				if (neighbor_x < 0 || neighbor_x >= image_h || 
						neighbor_y < 0 || neighbor_y >= image_w) {
						continue;
				}

				// 计算空间距离权重（基于坐标偏移）
				double spatialDist = row_d * row_d + col_d * col_d;
				double spatialWeight = exp(-spatialDist / (2 * sigmaSpace * sigmaSpace));

				// 计算像素值差异权重（基于原始图像）
				double intensityDiff = img[i][j] - img[neighbor_x][neighbor_y];
				double intensityWeight = exp(-(intensityDiff * intensityDiff) / (2 * sigmaColor * sigmaColor));

				// 总权重 = 空间权重 * 值权重
				double totalWeight = spatialWeight * intensityWeight;

				weightSum += totalWeight;
				filterValue += totalWeight * img[neighbor_x][neighbor_y];
			}
		}

		// 归一化并写入输出缓冲区
		if (weightSum > 1e-6) { // 避免除以零
				map_image[i][j] = (uint8)(filterValue / weightSum);
		} else {
				map_image[i][j] = img[i][j]; // 权重和过小，保留原值
		}
	}
}

// 将结果从 map_image 复制回原始图像
for (int i = 0; i < image_h; i++) {
		for (int j = 0; j < image_w; j++) {
				img[i][j] = map_image[i][j];
		}
}
}

// 3x3膨胀
AT_ITCM_SECTION_INIT( void dilate(uint8(*img)[image_w],  uint8(*output_img)[image_w],int kernel_size) ) {
	//clear map_image
	int offset = kernel_size / 2;

	for (int x = offset; x < image_h - offset; x++) 
	{
		for (int y = offset; y < image_w -offset; y++) 
		{
			uint8 max_value = 0;
			for(int dx = -offset; dx <= offset; dx++)
				for(int dy = -offset; dy <= offset; dy++)
					if(img[x + dx][y+dy] >= max_value) max_value = img[x + dx][y+dy];
			output_img[x][y] = max_value;
		}	
	}

}
// 3x3腐蚀
AT_ITCM_SECTION_INIT( void erode(uint8(*img)[image_w], uint8(*output_img)[image_w], int kernel_size) ){
	int offset = kernel_size / 2;
	for (int x = offset; x < image_h - offset; x++) 
    {
        for (int y = offset; y < image_w - offset; y++) 
        {
            uint8 min_value = 255; // 每个像素处理前重置为255
            for (int dx = -offset; dx <= offset; dx++)
            {
                for (int dy = -offset; dy <= offset; dy++)
                {
                    if (img[x + dx][y + dy] < min_value)
                    {
                        min_value = img[x + dx][y + dy];
                    }
                }
            }
            output_img[x][y] = min_value;
        }
    }

}

AT_ITCM_SECTION_INIT( void opening_process(uint8(*img)[image_w], int kernel_size) )
{
    
	erode(img, map_image, kernel_size);
	dilate(map_image, img, kernel_size);
}
AT_ITCM_SECTION_INIT( void closing_process(uint8(*img)[image_w], int kernel_size) )
{
    
	dilate(img, map_image, kernel_size);
	erode(map_image, img, kernel_size);
}
/* kernel = [1/16 1/8 1/16]
						[1/8  1/4 1/8]
						[1/16 1/8 1/16]	*/
//image_blur(mt9v03x_image, bin_image);
AT_ITCM_SECTION_INIT(void image_blur(uint8(*img0)[image_w], uint8(*img1)[image_w]))
{
	for (int x = 1; x < image_h - 1; x++)
	{
			for (int y = 1; y < image_w -1; y++)
			{
					img1[x][y] = (1 * img0[x-1][y-1] + 2 * img0[x][y-1] + 1 * img0[x+1][y-1] +
												2 * img0[x-1][y]	 + 4 * img0[x][y]		+ 2 * img0[x+1][y]	 +
												1 * img0[x-1][y+1] + 2 * img0[x][y+1] + 1 * img0[x+1][y+1]) / 16;
			}
	}
//	sobel(abandoned)
//	int gx, gy;
//	for (int x = 1; x < image_h - 1; x++) {
//			for (int y = 1; y < image_w -1; y++) {
//				gx =(-1 * img0[x-1][y-1] + 1 * img0[x+1][y-1] + 
//						 -2 * img0[x-1][y]   + 2 * img0[x+1][y]   +
//						 -1 * img0[x-1][y+1] + 1 * img0[x+1][y-1]  ) / 4;
//				gy =(	1 * img0[x-1][y-1] +  2 * img0[x]  [y-1] + 
//							1 * img0[x+1][y-1] +  2 * img0[x-1][y+1] +
//						 -2 * img0[x]  [y+1] + -1 * img0[x+1][y+1]  ) / 4;
//				img1[x][y] = (abs(gx) + abs(gy)) / 2;
//			}
//	}
}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w]	图像首地址
函数返回：无
修改时间：2022年9月8日
备    注：
example： image_draw_rectan(bin_image);
 */
AT_ITCM_SECTION_INIT(void image_draw_rectan(uint8(*image)[image_w]))
{

	uint8 i = 0;
	for (i = 0; i < image_h; i++)
	{
		image[i][0] = 0;
		image[i][1] = 0;
		image[i][image_w - 1] = 0;
		image[i][image_w - 2] = 0;

	}
	for (i = 0; i < image_w; i++)
	{
		image[0][i] = 0;
		image[1][i] = 0;
		image[image_h-1][i] = 0;
	}
}
AT_ITCM_SECTION_INIT(void img_exposure(uint8(*img)[image_w], int _start_row, int _end_row, float exposure_factor))
{
	int start_row,end_row;
	if(_start_row < _end_row)
	{
		start_row = _start_row;
		end_row = _end_row;
	}
	else
	{
		start_row = _end_row;
		end_row = _start_row;
	}

	for (int y =start_row; y < end_row; y++) {    // 遍历下半部分的行（y轴）
			for (int x = 0; x < image_w; x++) {      // 遍历所有列（x轴）
					// 乘法调整亮度（非线性）
				int new_value = img[y][x] * exposure_factor;
				new_value = (new_value < 0) ? 0 : (new_value > 255) ? 255 : new_value;
				img[y][x] = new_value;
			}
	}
	
	
}

uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
/**
 * @brief 从中向两边寻找两个边界点作为八邻域循环的起始点
 * @param uint8 start_row 起始的y坐标
 *  @see  get_start_point(image_h-2)
 * @return 返回说明
 * 		返回0	:未找到起始点
 * 		返回1	:找到起始点
 */
AT_ITCM_SECTION_INIT(uint8 get_start_point(uint8 start_row, uint8 end_row))
{
	uint8 i = 0,l_found = 0,r_found = 0;
	uint8 start = 0, end = 0;
	if(start_row > end_row)
	{
		start = start_row;
		end = end_row;
	}
	else
	{
		start = end_row;
		end = start_row;
	}
	for(uint8 row = start; row >= end; row--)
	{
		start_point_l[0] = 0;//x
		start_point_l[1] = 0;//y

		start_point_r[0] = 0;//x
		start_point_r[1] = border_max;//y
		//从中间找起点
		for (i = image_w / 2; i > border_min; i--)
		{
			start_point_l[0] = i;//x
			start_point_l[1] = row;//y
			if (bin_image[row][i] == white_pixel && bin_image[row][i - 1] == black_pixel)
			{
				//printf("找到左边起点image[%d][%d]\n", start_row,i);
				l_found = 1;
				break;
			}
		}
		for (i = image_w / 2; i < border_max; i++)
		{
			start_point_r[0] = i;//x
			start_point_r[1] = row;//y
			if (bin_image[row][i] == white_pixel && bin_image[row][i + 1] == black_pixel)
			{
				//printf("找到右边起点image[%d][%d]\n",start_row, i);
				r_found = 1;
				break;
			}
		}
		
		if(l_found && r_found)
		{
			//tft180_show_string(0, 90, "f_sp");
			return 1;
		}
	}
	
	return 0;//not found
	
}
int b_and_w=0;
int zebra_flag=0;
bool zebra_detect=false;
//斑马线判段
bool zebra_judge(void)
{
	zebra_flag=0;
	for(int i=image_h - 3;i > image_h - 35;i--)
	{
		b_and_w = 0;
		for(int j = l_border[i] + 1;j <= r_border[i] - 1;j ++)
		{
			if(bin_image[i][j] == 255 && bin_image[i][j+1] == 0)
				b_and_w++;
		}
		if(b_and_w >= 3)
		{
			//tft180_draw_line(1, i, 120 , i, RGB565_RED); 
			zebra_flag++;
		}
	}
	if(zebra_flag>=2)
	{
		system_delay_ms(600);
		return true;
	}
	else
	{
		return false;
	}
}


uint8 round_state = 0;
uint16 no_center_line_count = 0;

/**
 * @brief 最终处理函数
 * @param void
 *  @see image_process()
 * @return void
 */
AT_ITCM_SECTION_INIT(void image_process(void))
{
	uint16 i;
	uint8 highest = 0;//定义一个最高行, 这里的最高指的是y值的最小
	Get_image(mt9v03x_image);
//	image_blur(mt9v03x_image, original_image);//高斯滤波
	//img_exposure(original_image, image_h - 40, image_h, 1.07);
	//add_brightness(original_image, image_h - 30, image_h - 1, 40);
	//bilateralFilter(original_image, 21, 10, 10);
	turn_to_bin();//二值化
//	opening_process(bin_image, 3);
//	closing_process(bin_image, 3);
	
	image_not_so_much_filter(bin_image);//类似腐蚀和膨胀的滤波
	image_draw_rectan(bin_image);//卷积后的处理
	
	//清零
	data_stastics_l = 0;
	data_stastics_r = 0;
	uint8 start_row = 2;//从图像下往上数第几行
	uint8 end_row = 2; //从图像下往上数第几行
	//找到起点了, 再执行八领域, 没找到就不运行, 中线保持不变
	if (get_start_point(image_h - start_row, image_h - end_row))
	{
		no_center_line_count = 0;
		search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);
		get_left(data_stastics_l);
		get_right(data_stastics_r);
		if(!zebra_detect && cnt_01s > 20 &&	!mcx_find_flag)
			zebra_detect = zebra_judge(); 
		//if(iscircle(l_border, r_border, bin_image))
		circletemp(bin_image, l_border, r_border);
		fill_line(l_border, r_border, bin_image, l_index, r_index);
		
//		if(find_circle == false)
//		{
//			lost_line(l_border, r_border, data_stastics_l, data_stastics_r, points_l, points_r);//判断是否丢线
//			if(left_down_lost + right_down_lost == 2)
//			{
//				//判断十字路口
//				cross_fill(bin_image,l_border, data_stastics_l, data_stastics_r, r_border, 1);
//				
//			}
//			else if(left_up_lost + right_up_lost == 2)
//			{
//				//判断十字路口
//				cross_fill(bin_image,l_border, data_stastics_l, data_stastics_r, r_border, 2);
//			}
//			else if(left_down_lost + left_up_lost == 2)
//			{
//				//判断十字路口
//				cross_fill(bin_image,l_border, data_stastics_l, data_stastics_r, r_border, 3);
//			}
//			else if(right_down_lost + right_up_lost == 2)
//			{
//				//判断十字路口
//				cross_fill(bin_image,l_border, data_stastics_l, data_stastics_r, r_border, 4);
//			}
//		}
		
		
		
		//求中线
		int16 meet_right = 0;
		int16 meet_left = 0;
		int16 rec = 0;
		for (i = 0; i < image_h - 1; i++)
		{
			center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
			if(l_border[i] >= border_max - 7) 
				meet_right = i;
			else if(r_border[i] <= border_min + 7)
				meet_left = i;
			
		}
		if(meet_right)
			for(i = 0; i < meet_right; i ++)
				center_line[i] = border_max - 3;
		else if(meet_left)
			for(i = 0; i < meet_left; i ++)
				center_line[i] = border_min + 3;
		
		
	}
	//画边线和中线
	if(!zebra_detect)
		for (int i = 0; i < image_h - 1; i++)
		{
			tft180_draw_point((uint8)((float)l_border[i]/188*160) 	 , i, my_RED);
			tft180_draw_point((uint8)((float)r_border[i]/188*160) 	 , i, my_RED);
			tft180_draw_point((uint8)((float)center_line[i]/188*160) , i, my_BLUE);
		}
	

}

/*
//显示图像
ips154_displayimage032_zoom(bin_image[0], image_w, image_h, image_w, image_h,0,0);

	//根据最终循环次数画出边界点
	for (i = 0; i < data_stastics_l; i++)
	{
		ips154_drawpoint(points_l[i][0]+2, points_l[i][1], uesr_BLUE);//显示起点
	}
	for (i = 0; i < data_stastics_r; i++)
	{
		ips154_drawpoint(points_r[i][0]-2, points_r[i][1], uesr_RED);//显示起点
	}

	for (i = hightest; i < image_h-1; i++)
	{
		center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
		//求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
		//当然也有多组边线的找法，但是个人感觉很繁琐，不建议
		ips154_drawpoint(center_line[i], i, uesr_GREEN);//显示起点 显示中线	
		ips154_drawpoint(l_border[i], i, uesr_GREEN);//显示起点 显示左边线
		ips154_drawpoint(r_border[i], i, uesr_GREEN);//显示起点 显示右边线
	}


}

*/



/*

这里是起点（0.0）***************——>*************x值最大
************************************************************
************************************************************
************************************************************
************************************************************
******************假如这是一副图像*************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
***********************************************************
y值最大*******************************************(188.120)

*/


