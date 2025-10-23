//-------------------------------------------------------------------------------------------------------------------
//  简介:八邻域图像处理

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "zf_common_headfile.h"
int eresting=1;
float border = 94;
int buzzer = 0;
int cross_sum = 0;
int sum_island = 0;
//island 1为左环岛，2为右环岛
int island=0;
/*******单边桥变量设置***********/
SingleBridgeState  BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;
int jump_position = 0;
int stop_position = 0;
int monotonicity_line_l =0;
int monotonicity_line_r =0;
int string_not = 0;
int jump_h = 40;
int m=0;
int n=0;
int s=0;
int c=0;
int v=0;
int e=0;
int f=0;
int q=0;
int j=0;
int u=0;
/*
函数名称：int my_limit(int num,int value)
功能说明：两端限幅
参数说明：
函数返回：a <= c <= b
修改时间：2025年3月14日
备    注：
example：  my_auu(c,b,a)；
 */
int my_auu(int c,int b,int a)
 {
     if(c > b)
             return b;
     else if(c < a)
             return a;
     else
         return c;
 }
/*
函数名称：int my_limit(int num,int value)
功能说明：限幅
参数说明：
函数返回：限幅值
修改时间：2025年3月14日
备    注：
example：  my_abslimit( num,value)；
 */
int my_limit(int num,int value)
{
    if(value < num)
        return 0;
    else
        return value;
}
/*
函数名称：int my_abs(int value)
功能说明：求绝对值
参数说明：
函数返回：绝对值
修改时间：2025年1月5日
备    注：
example：  my_abs( x)；
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int limit_a_b(int x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
函数名称：int16 limit(int16 x, int16 y)
功能说明：求x,y中的最小值
参数说明：
函数返回：返回两值中的最小值
修改时间：2025年1月5日
备    注：
example：  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}


/*变量声明*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;//图像分割阈值
//------------------------------------------------------------------------------------------------------------------
//  @brief      获得一副灰度图像
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num     1   //1就是不压缩，2就是压缩一倍
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
//------------------------------------------------------------------------------------------------------------------
//  @brief     动态阈值
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
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
    uint8 MinValue=0, MaxValue=0;
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
//------------------------------------------------------------------------------------------------------------------
//  @brief      图像二值化，这里用的是大津法二值化。
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//图像数组
void turn_to_bin(void)
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


/*
函数名称：void get_start_point(uint8 start_row)
功能说明：寻找两个边界的边界点作为八邻域循环的起始点
参数说明：输入任意行数
函数返回：无
修改时间：2025年1月5日
备    注：
example：  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//左边起点的x，y值
uint8 start_point_r[2] = { 0 };//右边起点的x，y值
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //清零
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //从中间往左边，先找起点
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("找到左边起点image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("找到右边起点image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else if(l_found == 0 && r_found)
    {
        for (i = image_w * 3 / 4; i > border_min; i--)
        {
            start_point_l[0] = i;//x
            start_point_l[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
            {
                //printf("找到左边起点image[%d][%d]\n", start_row,i);
                l_found = 1;
                break;
            }
        }
        if(l_found&&r_found)return 1;
        else return 0;
    }
    else if(l_found && r_found == 0)
    {
        for (i = image_w / 4; i < border_max; i++)
        {
            start_point_r[0] = i;//x
            start_point_r[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
            {
                //printf("找到右边起点image[%d][%d]\n",start_row, i);
                r_found = 1;
                break;
            }
        }
        if(l_found&&r_found)return 1;
        else return 0;
    }
    else {
        //printf("未找到起点\n");
        return 0;
    }
}

/*
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2025年1月5日
备    注：
example：
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */

 //存放点的x，y坐标
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//左线
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//右线
uint16 dir_r[(uint16)USE_num] = { 0 };//用来存储右边生长方向
uint16 dir_l[(uint16)USE_num] = { 0 };//用来存储左边生长方向
uint16 data_stastics_l = 0;//统计左边找到点的个数
uint16 data_stastics_r = 0;//统计右边找到点的个数
uint8 hightest = 0;//最高点
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

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

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
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
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
//            printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
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
/*
函数名称：void get_left(uint16 total_L)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_L ：找到的点的总数
函数返回：无
修改时间：2025年1月5日
备    注：
example： get_left(data_stastics_l );
 */
uint8 l_border[image_h];//左线数组
uint8 r_border[image_h];//右线数组
uint16 l_index[image_h];
uint16 r_index[image_h];//映射关系,映出在原爬线上的位置
uint8 center_line[image_h];//中线数组
// 只有X边界
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

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
    h = image_h - 2;
    //左边
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0]+1;
            l_index[h] = j;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
        if (h == 0) 
        {
            break;//到最后一行退出
        }
    }
}
/*
函数名称：void get_right(uint16 total_R)
功能说明：从八邻域边界里提取需要的边线
参数说明：
total_R  ：找到的点的总数
函数返回：无
修改时间：2025年1月5日
备    注：
example：get_right(data_stastics_r);
 */
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
    h = image_h - 2;
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
#define threshold_max   255*5//此参数可根据自己的需求调节
#define threshold_min   255*2//此参数可根据自己的需求调节
void image_filter(uint8(*bin_image)[image_w])//形态学滤波，简单来说就是膨胀和腐蚀的思想
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

                bin_image[i][j] = 255;//白  可以搞成宏定义，方便更改

            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//黑

            }

        }
    }

}

/*
函数名称：void image_draw_rectan(uint8(*image)[image_w])
功能说明：给图像画一个黑框
参数说明：uint8(*image)[image_w] 图像首地址
函数返回：无
修改时间：2025年1月5日
备    注：
example： image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
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
        //image[image_h-1][i] = 0;

    }
}
/*补线巡线函数
函数名称：void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

功能说明：八邻域正式开始找右边点的函数，输入参数有点多，调用的时候不要漏了，这个是左右线一次性找完。
参数说明：
break_flag_r            ：最多需要循环的次数
(*image)[image_w]       ：需要进行找点的图像数组，必须是二值图,填入数组名称即可
                       特别注意，不要拿宏定义名字作为输入参数，否则数据可能无法传递过来
*l_stastic              ：统计左边数据，用来输入初始数组成员的序号和取出循环次数
*r_stastic              ：统计右边数据，用来输入初始数组成员的序号和取出循环次数
l_start_x               ：左边起点横坐标
l_start_y               ：左边起点纵坐标
r_start_x               ：右边起点横坐标
r_start_y               ：右边起点纵坐标
hightest                ：循环结束所得到的最高高度
函数返回：无
修改时间：2025年1月5日
备    注：
example：
    fill_search_l_r((uint16)USE_num,image,&data_stastics_l_fill, &data_stastics_r_fill,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest_fill);
 */

 //存放点的x，y坐标
uint16 points_l_fill[(uint16)USE_num][2] = { {  0 } };//左线补线
uint16 points_r_fill[(uint16)USE_num][2] = { {  0 } };//右线补线
uint16 dir_r_fill[(uint16)USE_num] = { 0 };//用来存储右边补线生长方向
uint16 dir_l_fill[(uint16)USE_num] = { 0 };//用来存储左边补线生长方向
uint16 data_stastics_l_fill = 0;//统计左边补线找到点的个数
uint16 data_stastics_r_fill = 0;//统计右边补线找到点的个数
uint8 hightest_fill = 0;//补线最高点
void fill_search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest_fill)
{

    uint8 i = 0, j = 0;

    //左边变量
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//统计左边
    //定义八个邻域
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是顺时针

    //右边变量
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//中心坐标点
    uint8 index_r = 0;//索引下标
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//统计右边
    //定义八个邻域
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //这个是逆时针

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

        //左边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_l_fill[l_data_statics][0] = center_point_l[0];//x
        points_l_fill[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//索引加一

        //右边
        for (i = 0; i < 8; i++)//传递8F坐标
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //中心坐标点填充到已经找到的点内
        points_r_fill[r_data_statics][0] = center_point_r[0];//x
        points_r_fill[r_data_statics][1] = center_point_r[1];//y

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
                dir_l_fill[l_data_statics - 1] = (i);//记录生长方向
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
        if ((points_r_fill[r_data_statics][0]== points_r_fill[r_data_statics-1][0]&& points_r_fill[r_data_statics][0] == points_r_fill[r_data_statics - 2][0]
            && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 1][1] && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 2][1])
            ||(points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 2][0] && points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 3][0]
                && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 2][1] && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 3][1]))
        {
            //printf("三次进入同一个点，退出\n");
            break;
        }
        if (my_abs(points_r_fill[r_data_statics][0] - points_l_fill[l_data_statics - 1][0]) < 2
            && my_abs(points_r_fill[r_data_statics][1] - points_l_fill[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n左右相遇退出\n");
            *hightest_fill = (points_r_fill[r_data_statics][1] + points_l_fill[l_data_statics - 1][1]) >> 1;//取出最高点
            //printf("\n在y=%d处退出\n",*hightest);
            break;
        }
        if ((points_r_fill[r_data_statics][1] < points_l_fill[l_data_statics - 1][1]))
        {
//            printf("\n如果左边比右边高了，左边等待右边\n");
            continue;//如果左边比右边高了，左边等待右边
        }
        if (dir_l_fill[l_data_statics - 1] == 7
            && (points_r_fill[r_data_statics][1] > points_l_fill[l_data_statics - 1][1]))//左边比右边高且已经向下生长了
        {
            //printf("\n左边开始向下了，等待右边，等待中... \n");
            center_point_l[0] = (uint8)points_l_fill[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l_fill[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//索引加一

        index_r = 0;//先清零，后使用
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//先清零，后使用
            temp_r[i][1] = 0;//先清零，后使用
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
                dir_r_fill[r_data_statics - 1] = (i);//记录生长方向
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
/*
函数名称：void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
功能说明：从八邻域补线边界里提取需要的边线
参数说明：
total_L ：左边找到的点的总数
total_R ：右边找到的点的总数
h        ：初始行数
函数返回：无
修改时间：2025年1月5日
备    注：
example： get_direction_fill(data_stastics_l_fill, data_stastics_r_fill, h);
 */

void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h_if = 0;
    //确定初始化范围
    if(h>image_h - 3)
    {
        h_if = image_h;
        h=image_h - 2;
    }
    else if(h<=0)
    {
        h_if = 0;
        h=0;
    }
    else
    {
        h_if = h;
    }
    //初始化
    for (i = 0;i<h_if;i++)
    {
        l_border[i] = border_min;
    }
    for (i = 0; i < h_if; i++)
    {
        r_border[i] = border_max;//右边线初始化放到最右边，左边线放到最左边，这样八邻域闭合区域外的中线就会在中间，不会干扰得到的数据
    }
    //左边
    for (j = 0; j < total_L; j++)
    {
        if (h == 0)
        {
            break;//到最后一行退出
        }
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l_fill[j][0]+1;
        }
        else continue; //每行只取一个点，没到下一行就不记录
        h--;
    }
    //右边
    for (j = 0; j < total_R; j++)
    {
        if (h == 0)
        {
            break;//到最后一行退出
        }
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r_fill[j][0] - 1;
        }
        else continue;//每行只取一个点，没到下一行就不记录
        h--;
    }
}

/**
* @brief 最小二乘法
* @param uint8 begin                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界首地址
*  @see CTest       Slope_Calculate(start, end, border);//斜率
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
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
* @brief 计算斜率截距
* @param uint8 start                输入起点
* @param uint8 end                  输入终点
* @param uint8 *border              输入需要计算斜率的边界
* @param float *slope_rate          输入斜率地址
* @param float *intercept           输入截距地址
*  @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
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

/**
 * @brief 左边丢线函数
 * @param uint8 *l_border     输入左边线地址
 * @param uint8 start         输入起始点
 * @param uint8 end           输入终止点
 * @see CTest       l_loss_judge(l_border,20 ,0)
 * @return 返回说明
 * -1 无效
 * 1  丢线
 * 0  不丢线
 */
int l_loss_judge(uint8 *l_border,uint8 start ,uint8 end)
{
    uint16 i;
    uint16 sum = 0;
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    if(start >= end)
        return -1;
    for(i = start;i <= end; i++)
    {
       if(l_border[i] <= border_min+2 )
           sum++;
    }
    if(sum >= (my_abs(start - end)/5*4))
        return 1;
    else
        return 0;
}

/**
 * @brief 右边丢线函数
 * @param uint8 *r_border     输入左边线地址
 * @param uint8 start         输入起始点
 * @param uint8 end           输入终止点
 * @see CTest       r_loss_judge(l_border,20 ,0)
 * @return 返回说明
 * -1 无效
 * 1  丢线
 * 0  不丢线
 */
int r_loss_judge(uint8 *r_border,uint8 start ,uint8 end)
{
    uint16 i;
    uint16 sum = 0;
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    if(start >= end)
        return -1;
    for(i = start;i <= end; i++)
    {
       if(r_border[i] >= border_max-2 )
           sum++;
    }
    if(sum >= (my_abs(start - end)/5*4))
        return 1;
    else
        return 0;
}

uint8 broken_line_x = 0;
uint8 broken_line_y = 0;
/**
 * @brief 断线寻找函数
 * @param uint8 dir                  输入0:从上往下寻，1：从下往上寻
 * @param uint8 start                输入起始点
 * @param uint8 *border              输入边界首地址
 * @see CTest       broken_line_judge(1,90,120,l_border);
 * @return 返回说明
 */
void broken_line_judge(int dir, uint8 start, uint8 end, uint8 *border)
{
    if(dir == 0)
    {
        for(uint16 i = (uint16)start ;i <= (uint16)end; i++)
        {
            if(my_abs(border[i] - border[i-1]) >= 4)
            {
                broken_line_x = border[i-1];
                broken_line_y = (uint8)(i-1);
                break;
            }
            if(i == (uint16)(end-2))
            {
                broken_line_x = -1;
                broken_line_y = -1;
                break;
            }
        }
    }
    if(dir == 1)
    {
        for(uint16 i = (uint16)end;i >= (uint16)start;i--)
        {
            if(my_abs(border[i] - border[i+1]) >= 4)
            {
                broken_line_x = border[i+1];
                broken_line_y = (uint8)(i+1);
                break;
            }
            if(i == (int)(start+2))
            {
                broken_line_x = -1;
                broken_line_y = -1;
                break;
            }
        }
    }

}
/**
 * @brief 单调判断函数
 * @param uint8* hightest            输入截至高度
 * @param uint8 *l_border            输入左边界首地址
 * @param uint8 *r_border            输入右边界首地址
 * @param uint16 total_num_l         输入左边循环总次数
 * @param uint16 total_num_r         输入右边循环总次数
 * @param uint16 *dir_l              输入左边生长方向首地址
 * @param uint16 *dir_r              输入右边生长方向首地址
 * @param uint16(*points_l)[2]       输入左边轮廓首地址
 * @param uint16(*points_r)[2]       输入右边轮廓首地址
 * @param uint16 *l_index            输入左边映射首地址
 * @param uint16 *r_index            输入右边映射首地址
 * @see CTest      monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
 * @return 返回说明
 * 1 单调
 * 0 不单调
 */
int monotonicity_l = -1;
int monotonicity_r = -1;
void monotonicity_line(uint8* hightest, uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
        uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],uint16 *l_index,uint16 *r_index)
{
    uint16 i;
    uint16 total_l = 0;
    uint16 total_r = 0;//生长方向总计
    uint16 arr_l = 0;
    uint16 arr_r = 0;//符合生长方向总计
    uint16 abb_l = 0;
    uint16 abb_r = 0;//断线个数总计
    uint16 acc_l = 0;
    uint16 acc_r = 0;//映射断点总计
    //左边单调性
    for (i = 1; i < total_num_l-20; i++)
    {
        if(points_l[i][0] == (uint16)l_border[points_l[i][1]] && points_l[i][1] <= 20)
            break;
        /*前二十行可能出现反光的不确定性因素*/
        if ((dir_l[i] == 4) || (dir_l[i] == 5) || (dir_l[i] == 6))
        {
            arr_l++;
        }
        total_l++;
    }
    /*单调性第一条件：生长方向（判断趋势设置反向拐弯为不单调）*/
    if(arr_l>=(total_l-5))
    {
        /*前二十行可能出现反光的不确定性因素*/
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*单调性第二条件：映射断点（判断趋势设置拐弯为不单调）*/
                }
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*单调性第三条件：断点（判断十字等断点为不单调）*/
                }
                if(i == 100)
                    /*后十行可能出现阴影的不确定性因素*/
                {
                    monotonicity_l = 1;
                    break;
                }
            }
        }
        else
            for (int i = 20; i < image_h-2; i++)
            {
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*单调性第二条件：映射断点（判断趋势设置拐弯为不单调）*/
                }
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*单调性第三条件：断点（判断十字等断点为不单调）*/
                }
                if(i == 100)
                    /*后十行可能出现阴影的不确定性因素*/
                {
                    monotonicity_l = 1;
                    break;
                }
            }
    }
    else monotonicity_l = 0;
    //右边单调性
    for (i = 1; i < total_num_r-20; i++)
    {
        if(points_r[i][0] == (uint16)r_border[points_r[i][1]] && points_r[i][1] <= 20)
            break;
        /*前二十行可能出现反光的不确定性因素*/
        if ((dir_r[i] == 4) || (dir_r[i] == 5) || (dir_r[i] == 6))
        {
            arr_r++;
        }
        total_r++;
    }
    /*单调性第一条件：生长方向（判断趋势设置反向拐弯为不单调）*/
    if(arr_r>=(total_r-5))
    {
        /*前二十行可能出现反光的不确定性因素*/
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*单调性第二条件：映射断点（判断趋势设置拐弯为不单调）*/
                }
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*单调性第三条件：断点（判断十字等断点为不单调）*/
                }
                if(i == 100)
                    /*后十行可能出现阴影的不确定性因素*/
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
        else
        {
            for (int i = 20; i < image_h-2; i++)
            {
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*单调性第二条件：映射断点（判断趋势设置拐弯为不单调）*/
                }
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*单调性第三条件：断点（判断十字等断点为不单调）*/
                }
                if(i == 100)
                    /*后十行可能出现阴影的不确定性因素*/
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
    }
    else monotonicity_r = 0;
}
/**
* @brief 十字状态机函数
* @param uint8(*image)[image_w]     输入二值图像
* @param uint8 *l_border            输入左边界首地址
* @param uint8 *r_border            输入右边界首地址
* @param uint16 total_num_l         输入左边循环总次数
* @param uint16 total_num_r         输入右边循环总次数
* @param uint16 *dir_l              输入左边生长方向首地址
* @param uint16 *dir_r              输入右边生长方向首地址
* @param uint16(*points_l)[2]       输入左边轮廓首地址
* @param uint16(*points_r)[2]       输入右边轮廓首地址
*  @see CTest       cross_fill(image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                                         uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint16 i;
    uint8 break_num_l = 0;
    uint8 break_num_r = 0;
    uint8 end_num_l = 0;
    uint8 end_num_r = 0;
    uint8 break_num_l_x = 0;
    uint8 break_num_r_x = 0;
    uint8 start, end;
    float slope_l_rate = 0, intercept_l = 0;
    if(cross_sum == 0)
    {
        if(((l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 80 ,100) == 0 && r_loss_judge(r_border, 50 ,70) == 1)
                || (l_loss_judge(l_border,80 ,100) == 0 && l_loss_judge(l_border, 50 ,70) == 1
                && r_loss_judge(r_border, 80 ,100) == 1 && r_loss_judge(r_border, 50 ,70) == 1)
                || (l_loss_judge(l_border,80 ,100) == 1 && l_loss_judge(l_border, 50 ,70) == 1
                && r_loss_judge(r_border, 80 ,100) == 0 && r_loss_judge(r_border, 50 ,70) == 1))
                && sum_island == 0
                )
        {
            cross_sum = 1;
        }
    }
    if(cross_sum == 1)//入十字
    {
        for (i = 1; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*第一判断条件：生长方向转折*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*第二判断条件：丢线*/
            {
                break_num_l = (uint8)points_l[i][1];//传递y坐标
                break;
            }
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /*第一判断条件：生长方向转折*/
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
                /*第二判断条件：丢线*/
            {
                end_num_l = (uint8)points_l[i][1];//传递y坐标
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*第一判断条件：生长方向转折*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*第二判断条件：丢线*/
            {
                break_num_r = (uint8)points_r[i][1];//传递y坐标
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                    /*第一判断条件：生长方向转折*/
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                         /*第二判断条件：丢线*/
            {
                end_num_r = (uint8)points_r[i][1];//传递y坐标
            }
        }
        for (int j = break_num_l; j<=end_num_l;j++)
        {
            for(int h=l_border[j]+1;h<=94;h++)
            {
                //扫描到白色，就进判断
                if (image[j][h-1] == 0 && image[i][h] == 255)
                {
                    cross_sum = 3;//进入十字圆环
                }
            }
        }
        for (int j = break_num_r; j<=end_num_r;j++)
        {
            for(int h=r_border[j]-1;h>=94;h--)
            {
                //扫描到白色，就进判断
                if (image[j][h+1] == 0 && image[i][h] == 255)
                {
                    cross_sum = 3;//进入十字圆环
                }
            }
        }
        if ((end_num_l == 0 && end_num_r == 0)||(end_num_l >= 110 && end_num_r >= 110)
                ||(break_num_l && break_num_r == 0 && end_num_l == 0 && end_num_r)
                ||(break_num_r && break_num_l == 0 && end_num_r == 0 && end_num_l)
                )
        {
            cross_sum = 2;
        }
        if (break_num_l && break_num_r == 0 && end_num_l && end_num_r == 0)
        {
            //计算斜率
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//斜率k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//截距b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
        if (break_num_r && break_num_l == 0 && end_num_r && end_num_l == 0)
         {
            //计算斜率
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//斜率k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//截距b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
         }
        if (break_num_l && break_num_r && end_num_l && end_num_r)
        {
            //计算斜率
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//斜率k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//截距b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            //计算斜率
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//斜率k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//截距b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }

    }
    if(cross_sum == 2)//出十字
    {
        for (i = 10; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*第一判断条件：生长方向转折*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*第二判断条件：丢线*/
            {
                break_num_l = (uint8)points_l[i][1]-5;//传递y坐标
                break;
            }
        }
        for (i = 10; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*第一判断条件：生长方向转折*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*第二判断条件：丢线*/
            {
                break_num_r = (uint8)points_r[i][1]-5;//传递y坐标
                break;
            }
        }
        if((break_num_l == 0 && break_num_r == 0)||(break_num_l >= 115 && break_num_r >= 115)
                ||(l_loss_judge(l_border,90 ,110) == 0 && r_loss_judge(r_border,90 ,110) == 0))
        {
            cross_sum = 0;
        }
        //计算斜率
        start = break_num_l - 10;
        start = (uint8)limit_a_b(start, 0, image_h);
        end = break_num_l - 5;
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
        for (i = break_num_l - 5; i < image_h - 1; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//限幅
        }

        //计算斜率
        start = break_num_r - 10;//起点
        start = (uint8)limit_a_b(start, 0, image_h);//限幅
        end = break_num_r - 5;//终点
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
        for (i = break_num_r - 5; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
    if(cross_sum == 3)//十字圆环
    {
        for (i = 1; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*第一判断条件：生长方向转折*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*第二判断条件：丢线*/
            {
                break_num_l = (uint8)points_l[i][1];//传递y坐标
                break;
            }
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /*第一判断条件：生长方向转折*/
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
                /*第二判断条件：丢线*/
            {
                end_num_l = (uint8)points_l[i][1];//传递y坐标
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*第一判断条件：生长方向转折*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*第二判断条件：丢线*/
            {
                break_num_r = (uint8)points_r[i][1];//传递y坐标
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                    /*第一判断条件：生长方向转折*/
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                         /*第二判断条件：丢线*/
            {
                end_num_r = (uint8)points_r[i][1];//传递y坐标
            }
        }
        for (int j = break_num_l; j<=end_num_l;j++)
        {
            for(int h=l_border[j]+1;h<=r_border[j]-1;h++)
            {
                //扫描到白色，就进判断
                if (image[j][h-1] == 0 && image[i][h] == 255)
                {
                    break_num_l_x = 1;
                }
                else
                    break_num_l_x = 0;
            }
        }
        for (int j = break_num_r; j<=end_num_r;j++)
        {
            for(int h=r_border[j]-1;h>=l_border[j]+1;h--)
            {
                //扫描到白色，就进判断
                if (image[j][h+1] == 0 && image[i][h] == 255)
                {
                    break_num_r_x = 1;
                }
                else
                    break_num_r_x = 0;
            }
        }
        if(break_num_l_x == 0 && break_num_r_x == 0)
        {
            cross_sum = 1;
        }
        //计算斜率
        start = end_num_l + 4;
        start = (uint8)limit_a_b(start, 0, image_h);
        end = end_num_l + 8;
        end = (uint8)limit_a_b(end, 0, image_h);
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
        for (i = 20; i < image_h - 1; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//限幅
        }

        //计算斜率
        start = end_num_r + 4;//起点
        start = (uint8)limit_a_b(start, 0, image_h);//限幅
        end = end_num_r + 8;//终点
        end = (uint8)limit_a_b(end, 0, image_h);//限幅
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
        for (i = 20; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
}

/**
* @brief 环岛状态机函数
* @param uint8(*image)[image_w]     输入二值图像
* @param uint8 *l_border            输入左边界首地址
* @param uint8 *r_border            输入右边界首地址
* @param uint16 total_num_l         输入左边循环总次数
* @param uint16 total_num_r         输入右边循环总次数
* @param uint16 *dir_l              输入左边生长方向首地址
* @param uint16 *dir_r              输入右边生长方向首地址
* @param uint16(*points_l)[2]       输入左边轮廓首地址
* @param uint16(*points_r)[2]       输入右边轮廓首地址
* @param uint16 *l_index            输入左边映射首地址
* @param uint16 *r_index            输入右边映射首地址
* @param int monotonicity_l         输入左边单调性
* @param int monotonicity_r         输入左边单调性
*  @see CTest       around_fill(image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index,monotonicity_l,monotonicity_r);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
   uint16 i;
   int black_line_sum = 0;
   int black_sum_1 = 0;
   int black_sum_2 = 0;
   uint8 break_num_l = 0;
   uint8 break_num_r = 0;
   uint8 end_num_l = 0;
   uint8 end_num_r = 0;
   uint8 start, end;
   int ap = 1;
   float slope_l_rate = 0, intercept_l = 0;
   //识别是否有环岛
   //左环岛
   if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
       //环岛第一条件:单调
   {
       broken_line_judge(1,*hightest,110,l_border);
//       for (uint16 w = broken_line_y + 15; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
//       {
//           if (r_border[w] > r_border[w + 10] && (r_border[w] != border_max || r_border[w + 10] != border_max))
//           {
//                    ap = 0;
//                    break;
//           }
//       }
       for (i = broken_line_y+5; i > *hightest; i--)
       {
           for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
           {
               //扫描到黑色，就进判断
               if (image[i][x] == 0 && image[i][x-1] == 255)
               {
                   black_sum_1 = 1;
               }
               //扫描到白色，就进判断
               if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
               {
                   black_sum_2 = 1;
               }
               //又扫描到黑色，就退出
               if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
               {
                   black_sum_1 = 0;
                   black_sum_2 = 0;
                   break;
               }
           }
           if(black_sum_2 == 1)
           {
               black_sum_1 = 0;
               black_sum_2 = 0;
               black_line_sum++;
           }
           if(i==15)//前十行可能有误差
               break;
       }
       if(black_line_sum>=5 /*环岛二条件:识别到环岛*/
               && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
               && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*环岛三条件:基础丢线判断*/
               && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
           /*环岛四条件:与十字和单边桥不冲突*/
               && image[broken_line_y-2][l_border[broken_line_y]-2] == 255)
           /*环岛五条件:与障碍物不冲突*/
       {
            sum_island = 1;//预测环岛标志位
            island = 1;//左环岛判断
            black_line_sum = 0;
       }
   }
   //右环岛
   if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
       //环岛第一条件:单调
   {
       broken_line_judge(1,*hightest,110,r_border);
//       for (uint16 w = broken_line_y + 15; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
//       {
//           if (l_border[w] < l_border[w + 10] && (l_border[w] != border_min || l_border[w + 10] != border_min))
//           {
//                    ap = 0;
//                    break;
//           }
//       }
       for (i = broken_line_y+5; i > *hightest; i--)
       {
           for (int x = (int)r_border[i]; x >= (int)l_border[i]; x--)
           {
               //扫描到黑色，就进判断
               if (image[i][x] == 0 && image[i][x+1] == 255)
               {
                   black_sum_1 = 1;
               }
               //扫描到白色，就进判断
               if (black_sum_1 == 1 && image[i][x+1] == 0 && image[i][x] == 255)
               {
                   black_sum_2 = 1;
               }
               //又扫描到黑色，就退出
               if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x+1] == 255)
               {
                   black_sum_1 = 0;
                   black_sum_2 = 0;
                   break;
               }
           }
           if(black_sum_2 == 1)
           {
               black_sum_1 = 0;
               black_sum_2 = 0;
               black_line_sum++;
           }
           if(i==15)//前十行可能有误差
               break;
       }
       if(black_line_sum>=5 /*环岛二条件:识别到环岛*/
               && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
               && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*环岛三条件:基础丢线判断*/
               && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
           /*环岛四条件:与十字和单边桥不冲突*/
               && image[broken_line_y-2][r_border[broken_line_y]+2] == 255)
           /*环岛五条件:与障碍物不冲突*/
       {
            sum_island = 1;//预测环岛标志位
            island = 2;//右环岛判断
            black_line_sum = 0;
       }
   }
    //左环岛
    if(island == 1)
    {
        //初见环岛
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,l_border);
            for (uint16 w = broken_line_y + 10; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
            {
                if(w<=15)
                {
                    continue;
                }
                else if(w-10 == my_auu((broken_line_y - 15),110,*hightest))
                {
                    break;
                }
                else
                {
                    if (r_border[w-10] > r_border[w] && (r_border[w-10] != border_max || r_border[w] != border_max))
                    {
                             ap = 0;
                             break;
                    }
                }
            }
            if(ap == 0)//不是环岛退出,防止出现缺口，类十字
            {
                island = 0;
                sum_island = 0;
            }
            if(broken_line_y >= 15 && ap == 1)
            {
                //计算斜率(根据一个点拉长补线)
                 start = broken_line_y+5;//起点
                 end = broken_line_y+10;//终点
                 calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
                 for (i = 1; i < broken_line_y+1; i++)
                 {
                     l_border[i] = slope_l_rate * (i)+intercept_l;
                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                 }
                 if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
                 {
                     for (i = 106; i > 15; i--)
                     {
                         //通过映射找点，找到进入下一个状态
                         if(points_l[l_index[i]][0]>points_l[l_index[i-7]][0] && points_l[l_index[i]][0]>points_l[l_index[i+7]][0]
                            && points_l[l_index[i-7]][0] != border_min)
                         {
                             sum_island = 2;
                         }
                     }
                 }
            }
        }
        //初入环岛
        if(sum_island == 2)
        {
            int dp = 0;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            if (!image[image_h - 5][5] && !image[image_h - 3][3])
            {
                dp = 1;
            }
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]
                         &&points_l[j][1] > points_l[j + 5][1]
                         &&points_l[j][1] >= points_l[j - 3][1]
                         &&points_l[j][1] >= points_l[j - 5][1])
                    {
                        vp = h;
                        break;
                    }
                }
            }
            for (i = 25; i < image_h - 15; i++)
            {
                if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
                   && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
                   && l_border[i-5] != border_min && l_border[i+5] != border_min)
                {
                    end_num_l = (uint8)i;
                }
                if(vp && end_num_l >= 60)
                    sum_island = 3;
            }
                //计算斜率（双点连接补线）
                slope_l_rate = (float)(118-end_num_l) / (border_min-l_border[end_num_l]);//斜率k=y/x
                intercept_l = 118 - slope_l_rate*border_min;//截距b=y-kx
                for (i = end_num_l; i < image_h - 1; i++)
                {
                    l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }

        }
        if(sum_island == 3)
        {
            uint16 h = 0;
            int temph_l = 0;
            for (h = image_h - 15; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    temph_l = h;
                    break;
                }
            }
            if (temph_l)
            {
                for (int i = l_index[temph_l - 2]; i > l_index[temph_l + 1]; i--)
                {

                    if (points_l[i][1] >= points_l[i + 3][1]
                        &&points_l[i][1] > points_l[i + 5][1]
                        &&points_l[i][1] > points_l[i + 7][1]
                        &&points_l[i][1] >= points_l[i - 3][1]
                        &&points_l[i][1] >= points_l[i - 5][1]
                        &&points_l[i][1] > points_l[i - 8][1]
                        &&points_l[i][0] > points_l[i - 5][0]
                        &&points_l[i][0] < points_l[i + 5][0])
                    {
                        break_num_l = (uint8)points_l[i][1];//传递y坐标
                        end_num_l  = (uint8)points_l[i][0];//传递x坐标
                        break;
                    }
                }
            }
            if((*hightest >= 30)&&(!image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))
            {
                sum_island = 4;
            }
            if(break_num_l && end_num_l)
            {
                //计算斜率
                slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);//斜率k=y/x
                intercept_l = 118 - slope_l_rate*186;//截距b=y-kx
                for (i = 1; i < image_h - 1; i++)
                {
                    r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                }
                for (i = 1; i < break_num_l; i++)
                {
                    l_border[i] = border_min;
                }
            }
        }
        if(sum_island == 4)
        {
            int g=0;
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    g=1;
                    break;
                }
            }
            for (i = image_h - 20; i > *hightest; i--)//断线判别法
            {
                if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>10)
                {
                    if(i>=30 && i<=105 && g)
                        sum_island = 5;
                }
            }
        }
        if(sum_island == 5)
        {
            for (uint16 w = image_h - 15; w > *hightest; w--)//断线判别法
            {
                if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
                {
                    break_num_l = (uint8)w;//传递y坐标
                    break;
                }
            }
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    end_num_l = (uint8)points_l[i][1];//传递y坐标
                    break;
                }
            }
            if(r_loss_judge(r_border, 95 ,110) == 1 && r_loss_judge(r_border, 80 ,95) == 1)
                sum_island = 6;
            //计算斜率
            slope_l_rate = (float)(break_num_l-20) / (r_border[break_num_l]-0);//斜率k=y/x
            intercept_l = 20 - slope_l_rate*0;//截距b=y-kx
            for (i = 1; i < break_num_l-3; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
            for (i = 1; i < end_num_l; i++)
            {
                l_border[i] = border_min;
            }
        }
        //出圆环
        if(sum_island == 6)
        {
            int dp = 1;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 20)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]
                         &&points_l[j][1] > points_l[j + 5][1]
                         &&points_l[j][1] >= points_l[j - 3][1]
                         &&points_l[j][1] >= points_l[j - 5][1])
                    {
                        vp = h;
                        break;
                    }
                }
            }
            if(vp && r_loss_judge(r_border, 70 ,90) == 0
                    && r_loss_judge(r_border, 50 ,70) == 0)
                sum_island = 7;
            //计算斜率
            slope_l_rate = (float)(118-20) / (186-0);//斜率k=y/x
            intercept_l = 20 - slope_l_rate*0;//截距b=y-kx
            for (i = 1; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        //出环岛
        if(sum_island == 7)
        {
            uint16 h = 0;
            for (h = image_h - 25; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    break_num_l = (uint8)(h-2);//传递y坐标
                    end_num_l  = (uint8)l_border[h - 2];//传递x坐标
                    break;
                }
            }
            if ((!image[image_h - 10][image_w - 10] && !image[image_h - 5][image_w - 5]
                 && !image[image_h - 15][image_w - 15] && !image[image_h - 20][image_w - 15]
                 && !image[image_h - 10][10] && !image[image_h - 5][5]
                 && !image[image_h - 20][15] && !image[image_h - 15][15]) || h >= 110)
            {
                if(r_loss_judge(r_border, 100 ,110) == 0 && l_loss_judge(l_border, 100 ,110) == 0
                      && r_loss_judge(r_border, 90 ,100) == 0 && l_loss_judge(l_border, 90 ,100) == 0
                      && r_loss_judge(r_border, 80 ,90) == 0 && l_loss_judge(l_border, 80 ,90) == 0
                      && r_loss_judge(r_border, 60 ,70) == 0 && l_loss_judge(l_border, 60 ,70) == 0
                      && r_loss_judge(r_border, 70 ,80) == 0 && l_loss_judge(l_border, 70 ,80) == 0)
                {
                    sum_island = 0;
                    island = 0;
                }
            }
            //计算斜率（双点连接补线）
            slope_l_rate = (float)(118-break_num_l) / (2-l_border[break_num_l]);//斜率k=y/x
            intercept_l = 118 - slope_l_rate*2;//截距b=y-kx
            for (i = break_num_l-10; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
    }
    //右环岛
    if(island == 2)
    {
        //初见环岛
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,r_border);
            for (uint16 w = broken_line_y + 10; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
            {
                if(w<=15)
                {
                    continue;
                }
                else if(w-10 == my_auu((broken_line_y - 15),110,*hightest))
                {
                    break;
                }
                else
                {
                    if (l_border[w-10] < l_border[w] && (l_border[w-10] != border_min || l_border[w] != border_min))
                    {
                             ap = 0;
                             break;
                    }

                }
            }
            if(ap == 0)//不是环岛退出,防止出现缺口，类十字
             {
                 island = 0;
                 sum_island = 0;
             }
            if(broken_line_y >= 15 && ap == 1)
            {
                //计算斜率(根据一个点拉长补线)
                 start = broken_line_y+5;//起点
                 end = broken_line_y+10;//终点
                 calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
                 for (i = 1; i < broken_line_y+1; i++)
                 {
                     r_border[i] = slope_l_rate * (i)+intercept_l;
                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                 }
                 if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
                 {
                     for (i = 106; i > 15; i--)
                     {
                         //通过映射找点，找到进入下一个状态
                         if(points_r[r_index[i]][0]<points_r[r_index[i-7]][0] && points_r[r_index[i]][0]<points_r[r_index[i+7]][0]
                              && points_r[r_index[i-7]][0] != border_max)
                         {
                             sum_island = 2;
                         }
                     }
                 }
            }
        }
    //初入环岛
    if(sum_island == 2)
    {
        int dp = 0;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (!image[image_h - 5][image_w - 5] && !image[image_h - 3][image_w - 3])
        {
            dp = 1;
        }
        if (dp)
        {
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
                {
                    temph = h;
                    break;
                }
            }
        }
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
        for (i = 25; i < image_h - 15; i++)
        {
            if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
               && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
               && r_border[i-7] != border_max && r_border[i+7] != border_max)
            {
                end_num_r = (uint8)i;
            }
            if(vp && end_num_r >= 60)
                sum_island = 3;
        }
            //计算斜率（双点连接补线）
            slope_l_rate = (float)(118-end_num_r) / (border_max-r_border[end_num_r]);//斜率k=y/x
            intercept_l = 118 - slope_l_rate*border_max;//截距b=y-kx
            for (i = end_num_r-10; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
    }
    if(sum_island == 3)
    {
        uint16 h = 0;
        int temph_r = 0;
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
            {
                temph_r = h;
                break;
            }
        }
        if (temph_r)
        {
            for (int i = r_index[temph_r - 2]; i > r_index[temph_r + 1]; i--)
            {

                if (points_r[i][1] >= points_r[i + 3][1]
                    &&points_r[i][1] > points_r[i + 5][1]
                    &&points_r[i][1] > points_r[i + 7][1]
                    &&points_r[i][1] >= points_r[i - 3][1]
                    &&points_r[i][1] >= points_r[i - 5][1]
                    &&points_r[i][1] > points_r[i - 8][1]
                    &&points_r[i][0] < points_r[i - 5][0]
                    &&points_r[i][0] > points_r[i + 5][0])
                {
                    break_num_r = (uint8)points_r[i][1];//传递y坐标
                    end_num_r  = (uint8)points_r[i][0];//传递x坐标
                    break;
                }
            }
        }
        if((*hightest >= 30)&& (!image[image_h - 1][3] && !image[image_h - 3][3]))
        {
            sum_island = 4;
        }
        if(break_num_r && end_num_r)
        {
            //计算斜率
            slope_l_rate = (float)(break_num_r-118) / (end_num_r-2);//斜率k=y/x
            intercept_l = 118 - slope_l_rate*2;//截距b=y-kx
            for (i = 1; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            for (i = 1; i < break_num_r; i++)
            {
                r_border[i] = border_max;
            }
        }
    }
    if(sum_island == 4)
    {
        int g=0;
        for (i = 1; i < total_num_r; i++)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
            {
                g=1;
                break;
            }
        }
        for (i = image_h - 20; i > *hightest; i--)//断线判别法
        {
            if (l_border[i] > l_border[i - 3] && my_abs(l_border[i-3] - l_border[i])>10)
            {
                if(i>=30 && i<=105 && g)
                    sum_island = 5;
            }
        }
    }
    if(sum_island == 5)
    {
        for (uint16 w = image_h - 15; w > *hightest; w--)//断线判别法
        {
            if (l_border[w] > l_border[w - 3] && my_abs(l_border[w] - l_border[w-3])>10)
            {
                break_num_r = (uint8)w;//传递y坐标
                break;
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
            {
                end_num_r = (uint8)points_r[i][1];//传递y坐标
                break;
            }
        }
        if(l_loss_judge(l_border, 95 ,110) == 1 && l_loss_judge(l_border, 80 ,95) == 1)
            sum_island = 6;
        //计算斜率
        slope_l_rate = (float)(break_num_r-20) / (l_border[break_num_r]-188);//斜率k=y/x
        intercept_l = 20 - slope_l_rate*188;//截距b=y-kx
        for (i = 1; i < break_num_r-3; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
        for (i = 1; i < end_num_r; i++)
        {
            r_border[i] = border_max;
        }
    }
    //出圆环
    if(sum_island == 6)
    {
        int dp = 1;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (dp)
        {
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 20)
                {
                    temph = h;
                    break;
                }
            }
        }
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
        if(vp && l_loss_judge(l_border, 70 ,90) == 0
                && l_loss_judge(l_border, 50 ,70) == 0)
            sum_island = 7;
        //计算斜率
        slope_l_rate = (float)(118-20) / (0-188);//斜率k=y/x
        intercept_l = 20 - slope_l_rate*188;//截距b=y-kx
        for (i = 1; i < image_h - 1; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
    //出环岛
    if(sum_island == 7)
    {
        uint16 h = 0;
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
            {
                break_num_r = (uint8)(h-2);//传递y坐标
                end_num_r  = (uint8)r_border[h - 2];//传递x坐标
                break;
            }
        }
        if ((!image[image_h - 10][image_w - 10] && !image[image_h - 5][image_w - 5]
             && !image[image_h - 15][image_w - 15] && !image[image_h - 20][image_w - 15]
             && !image[image_h - 10][10] && !image[image_h - 5][5]
             && !image[image_h - 20][15] && !image[image_h - 15][15]) || h >= 110)
        {
            if(r_loss_judge(r_border, 100 ,110) == 0 && l_loss_judge(l_border, 100 ,110) == 0
                  && r_loss_judge(r_border, 90 ,100) == 0 && l_loss_judge(l_border, 90 ,100) == 0
                  && r_loss_judge(r_border, 80 ,90) == 0 && l_loss_judge(l_border, 80 ,90) == 0
                  && r_loss_judge(r_border, 60 ,70) == 0 && l_loss_judge(l_border, 60 ,70) == 0
                  && r_loss_judge(r_border, 70 ,80) == 0 && l_loss_judge(l_border, 70 ,80) == 0)
            {
                sum_island = 0;
                island = 0;
            }
        }
        //计算斜率（双点连接补线）
        slope_l_rate = (float)(118-break_num_r) / (186-r_border[break_num_r]);//斜率k=y/x
        intercept_l = 118 - slope_l_rate*186;//截距b=y-kx
        for (i = break_num_r-10; i < image_h - 1; i++)
        {
            r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
     }
   }
}
/**
* @brief 斑马线状态机函数
* @param uint8(*image)[image_w]     输入二值图像
* @param uint8 *l_border            输入左边界首地址
* @param uint8 *r_border            输入右边界首地址
*  @see CTest       cross_stop(image,l_border,r_border);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_stop(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border)
{
    uint8 start_point  = 80;
    uint8 end_point = 100;
    //变量标志位
    int banmaxian_kuandu;//斑马线宽度
    int banmaxian_hangshu;//斑马线行数
    int banmaxian_geshu;//斑马线个数（块）
    banmaxian_kuandu=0;
    banmaxian_hangshu=0;
    banmaxian_geshu=0;
    //从下往上扫描
    for (uint16 y = end_point; y >= start_point; y--)
    {
         //从右往左扫描
         for (int x = (int)(l_border[y]+1); x <= (int)(r_border[y]-1); x++)
        {
            int baidian_heng = 0;
            //扫描到黑色，就进判断
            if (image[y][x] == 0 && image[y][x-1] == 255)
            {
                for(int a=x+1;a<x+15;a++)//从黑色点向左侧扫描
                {
                    //找到白色点
                    if(image[y][a-1] == 0 && image[y][a] == 255)
                    {
                        //记录白色点的位置，跳出循环
                        baidian_heng=a;
                        break;
                    }
                }//斑马线宽度等于黑白点的差
                banmaxian_kuandu = baidian_heng - x;
                //斑马线的宽度在2~6之间认为它成立为斑马线黑色块
                if (banmaxian_kuandu >= 2 && banmaxian_kuandu <= 10)
                {
                    //斑马线黑色块++
                    banmaxian_geshu++;
                    //斑马线色块宽度清零，进行下一个黑色块的扫描计算
                    banmaxian_kuandu = 0;
                }
                //斑马线的宽度不在4~8之间不认为它成立为斑马线黑色块
                else
                {
                    //如果不满足对黑色块的认为要求就直接清零，去计算下一个黑色块
                    banmaxian_kuandu = 0;
                }
            }
        }
        //如果色块的个数在6~14之间则认为这一行的斑马线满足要求，在去扫下一行
        if (banmaxian_geshu >= 6 && banmaxian_geshu <= 14)
        {
            banmaxian_hangshu++;
        }
        else
            banmaxian_geshu = 0;
    }
    //如果有大于等于4行的有效斑马线
    if(banmaxian_hangshu >= 4 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
            && jump_position == 0 && sum_island == 0 && island == 0 && cross_sum == 0)
    {
        //斑马线标准位置1
        stop_position = 1;
    }
}
/**
* @brief 单边桥状态机函数
* @param uint8(*image)[image_w]     输入二值图像
* @param uint8 *l_border            输入左边界首地址
* @param uint8 *r_border            输入右边界首地址
* @param uint8 *center_line         输入中边界首地址
* @param uint16 total_num_l         输入左边循环总次数
* @param uint16 total_num_r         输入右边循环总次数
* @param uint16 *dir_l              输入左边生长方向首地址
* @param uint16 *dir_r              输入右边生长方向首地址
* @param uint16(*points_l)[2]       输入左边轮廓首地址
* @param uint16(*points_r)[2]       输入右边轮廓首地址
* @param int8* hightest             输入截至高度
* @param uint16 *l_index            输入左边映射首地址
* @param uint16 *r_index            输入右边映射首地址
*  @see CTest       bridge_fill(bin_image,l_border, r_border, center_line, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line,
        uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r,
        uint16(*points_l)[2], uint16(*points_r)[2],uint8* hightest,uint16 *l_index,uint16 *r_index)
{
    int h = 0;
    int w = 0;
    int white_line1 = 0;
    int white_line2 = 0;
    int blake_line = 0;
    int cross1 = 0;
    int cross2 = 0;//拐点判断
    int long_start_l = 0;
    int long_end_l = 0;
    int long_start_r = 0;
    int long_end_r = 0;
    int loss = 0;//拐点个数判断
    int bridge_number = 0;
    for (int i = 15; i < image_h-10; i++)
    {
        if(my_abs(l_border[i] - l_border[i+3]) >= 6 && l_border[i]>l_border[i+3]
             && l_border[i]<r_border[i] && l_border[i] < 100)
            loss++;//拐点个数判断
        if(my_abs(r_border[i] - r_border[i+3]) >= 6 && r_border[i]<r_border[i+3]
            && l_border[i]<r_border[i] && r_border[i] > 80 )
            loss++;
    }
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        if(*hightest <= 15)//直线判断
        {
            for (int i = image_h-20; i>=15 ;i--)
            {
                for (int x = (l_border[i]+1); x <= (r_border[i]-1); x++)
                {
                    if(image[i][x] == 0)
                    {
                        blake_line++;
                        break;
                    }
                    if(image[i][x] == 255)//判断行
                    {
                        white_line1++;
                    }
                }
                for (int x = (l_border[i-3]+1); x <= (r_border[i-3]-1); x++)
                {
                    if(image[i-3][x] == 0)
                    {
                        blake_line++;
                        break;
                    }
                    if(image[i-3][x] == 255)//单边桥行
                    {
                        white_line2++;
                    }
                }
                if(blake_line >=3)
                {
                    blake_line = 1;
                    break;
                }
                else
                    blake_line = 0;
                if(white_line2 <= ((white_line1-3)/2))
                {
                    h=i-3;
                    for (int j = l_index[h-2]; j > l_index[h+5]; j--)
                    {
                        if (points_l[j][1] >= points_l[j + 1][1]
                            &&points_l[j + 1][0] >= points_l[j][0]
                            &&points_l[j][1] > points_l[j + 2][1]
                            &&points_l[j][0] <= points_l[j + 2][0]
                            &&points_l[j][1] == points_l[j - 3][1]
                            &&points_l[j][0] > points_l[j - 3][0]
                            &&points_l[j][0] > points_l[j - 5][0]
                            &&points_l[j][1] >= points_l[j - 5][1])
                        {
                            cross2=1;//左拐点
                            h=points_l[j][1];
                        }
                    }
                    for (int j = r_index[h-2]; j >= r_index[h+5]; j--)
                    {
                        if (points_r[j][1] >= points_r[j + 1][1]
                            &&points_r[j + 1][0] <= points_r[j][0]
                            &&points_r[j][1] > points_r[j + 2][1]
                            &&points_r[j][0] >= points_r[j + 2][0]
                            &&points_r[j][1] == points_r[j - 3][1]
                            &&points_r[j][0] < points_r[j - 3][0]
                            &&points_r[j][0] < points_r[j - 5][0]
                            &&points_r[j][1] >= points_r[j - 5][1])
                        {
                            cross2=2;//右拐点
                            h=points_r[j][1];
                        }
                    }
                    for (int j = h; j>=15 ;j--)
                    {
                       if(cross2==1 && l_border[j]==l_border[j-1])
                       {
                           for (w = l_index[j] + 5; w >= l_index[j]; w--)
                           {
                               if (dir_l[w] < 4)
                                   break;
                           }
                           if(w <= l_index[j]+1)
                           {
                               cross1++;
                               w=0;
                           }
                           else
                               w=0;
                       }
                       if(cross2==2 && r_border[j]==r_border[j-1])
                       {
                           for (w = r_index[j] + 5; w >= r_index[j]; w--)
                           {
                               if (dir_r[w] < 4)
                                   break;
                           }
                           if(w <= r_index[j]+1)
                           {
                               cross1++;
                               w=0;
                           }
                           else
                               w=0;
                       }
                    }
                    break;
                }
                else
                {
                    h=0;
                    white_line1 = 0;
                    white_line2 = 0;
                }
            }
            if(h>=35 && loss>=6 && blake_line == 0 && cross1>=5 && cross2>0)//60行是单边桥时进入，两个以上拐点
            {
                if(sum_island == 0 && island == 0 && jump_position == 0 && cross_sum == 0
                    &&l_loss_judge(l_border,(uint8)h ,110) == 0 && l_loss_judge(l_border,(uint8)h ,(uint8)(h+20)) == 0
                    && r_loss_judge(r_border,(uint8)h ,110) == 0 && r_loss_judge(r_border,(uint8)h ,(uint8)(h+20)) == 0)
                BridgeState = SINGLE_BRIDGE_ACTIVE;//进入单边桥标志位
            }
        }
    }
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
            for (int i = 30; i < image_h-1; i++)
            {
                if(i == 30)
                {
                    if(my_abs(l_border[i]-94)<=5 && my_abs(r_border[i]-94)>=10)
                    {
                        for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                        {
                            if(a == image_h-5)
                            {
                                BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//不是单边桥
                                break;
                            }
                            if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                                    image[a+2][l_border[a-3]] == 255 && image[a-3][l_border[a-3]-5] == 0)
                            {
                                long_start_l = i;
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }

                        }
                    }
                }
                if(i>30 && my_abs(l_border[i] - l_border[i+3]) >= 10 && long_start_l == 0 && long_end_l == 0)
                {
                    if(image[i][l_border[i+3]] == 255 && image[i+6][l_border[i+3]-5] == 0)
                    {
                        long_start_l = i+3;
                        for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                        {
                            if(a == image_h-5)
                            {
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }
                            if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                                    image[a+2][l_border[a-3]] == 255 && image[a-3][l_border[a-3]-5] == 0)
                            {
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }

                        }
                    }
                }
                if(i <= long_end_l && i >= long_start_l && long_start_l != 0 && long_end_l != 0)
                {
                    center_line[i] = l_border[i];
                    if(i+1 == long_end_l)
                    {
                        long_end_l = 0;
                        long_start_l = 0;
                    }
                }
            }
            for (int i = 30; i < image_h-1; i++)
            {
                if(i == 30)
                {
                    if(my_abs(r_border[i]-94)<=5 && my_abs(l_border[i]-94)>=10)
                    {
                         for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                         {
                             if(a == image_h-5)
                             {
                                 BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//不是单边桥
                                 break;
                             }
                             if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                                     image[a+2][r_border[a-3]] == 255 && image[a-3][r_border[a-3]+5] == 0)
                             {
                                 long_start_r = i;
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                         }
                    }
                }
                if(i>30 && my_abs(r_border[i] - r_border[i+3]) >= 10 && long_start_r == 0 && long_end_r == 0)
                {
                    if(image[i][r_border[i+3]] == 255 && image[i+6][r_border[i+3]+5] == 0)
                    {
                        long_start_r = i+3;
                         for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                         {
                             if(a == image_h-5)
                             {
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                             if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                                     image[a+2][r_border[a-3]] == 255 && image[a-3][r_border[a-3]+5] == 0)
                             {
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                         }
                    }
                }
                if(i <= long_end_r && i >= long_start_r && long_start_r != 0 && long_end_r != 0)
                {
                    center_line[i] = r_border[i];
                    if(i+1==long_end_r)
                    {
                        long_end_r = 0;
                        long_start_r = 0;
                    }
                }
            }
            if(bridge_number == 0)
            {
                cross2=0;
                for (int i = image_h-20; i>=50 ;i--)
                {
                    for (int x = (l_border[i]+1); x <= (r_border[i]-1); x++)
                    {
                        if(image[i][x] == 255)//判断行
                        {
                            white_line1++;
                        }
                    }
                    for (int x = (l_border[i-3]+1); x <= (r_border[i-3]-1); x++)
                    {
                        if(image[i-3][x] == 255)//单边桥行
                        {
                            white_line2++;
                        }
                    }
                    if(white_line2 <= ((white_line1-3)/2))
                    {
                        h=i-3;
                        for (int j = l_index[h-2]; j > l_index[h+5]; j--)
                        {
                            if (points_l[j][1] >= points_l[j + 1][1]
                                &&points_l[j + 1][0] >= points_l[j][0]
                                &&points_l[j][1] > points_l[j + 2][1]
                                &&points_l[j][0] <= points_l[j + 2][0]
                                &&points_l[j][1] == points_l[j - 3][1]
                                &&points_l[j][0] > points_l[j - 3][0]
                                &&points_l[j][0] > points_l[j - 5][0]
                                &&points_l[j][1] >= points_l[j - 5][1])
                            {
                                cross2=1;//左拐点
                                h=points_l[j][1];
                            }
                        }
                        for (int j = r_index[h-2]; j >= r_index[h+5]; j--)
                        {
                            if (points_r[j][1] >= points_r[j + 1][1]
                                &&points_r[j + 1][0] <= points_r[j][0]
                                &&points_r[j][1] > points_r[j + 2][1]
                                &&points_r[j][0] >= points_r[j + 2][0]
                                &&points_r[j][1] == points_r[j - 3][1]
                                &&points_r[j][0] < points_r[j - 3][0]
                                &&points_r[j][0] < points_r[j - 5][0]
                                &&points_r[j][1] >= points_r[j - 5][1])
                            {
                                cross2=2;//右拐点
                                h=points_r[j][1];
                            }
                        }
                        break;
                    }
                    else
                    {
                        white_line1 = 0;
                        white_line2 = 0;
                        h=0;
                    }
                }
                if(cross2 == 0
                   &&l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 0
                   && r_loss_judge(r_border,90 ,110) == 0 && r_loss_judge(r_border,60 ,80) == 0)
                BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//退出单边桥标志位
            }
        }
    }

/**
* @brief 跳跃状态机函数
* @param uint8(*image)[image_w]     输入二值图像
* @param uint8* hightest            输入截至高度
* @param uint8 *l_border            输入左边界首地址
* @param uint8 *r_border            输入右边界首地址
* @param uint8 monotonicity_l       输入左边单调性
* @param uint8 monotonicity_r       输入右边单调性
* @see CTest       jump_judge(bin_image,&hightest,l_border,r_border,monotonicity_l, monotonicity_r,(Encoder_Left-Encoder_Right)/2);
* @return 返回说明
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r,float now_velocity)
{
    if(jump_position == 0)
    {
        int blake_line = 0;
        jump_h = (int)(-0.1006*now_velocity+103.8);
        if(now_velocity >= 580)
        {
            jump_h = (int)(-0.1056*now_velocity+107.5);
            if(now_velocity >= 700)
            {
                jump_h = jump_h+1;
            }
            else if(now_velocity >= 650)
            {
                jump_h = jump_h-2;
            }
            else if(now_velocity >= 620)
            {
                jump_h = jump_h-1;
            }
        }
        else if(now_velocity > 550 && now_velocity < 580)
        {
            jump_h = jump_h+1;
        }
        else if(now_velocity < 400)
        {
            jump_h = jump_h+2;
        }
        my_auu(jump_h,110,30);
       if(*hightest>=(uint8)jump_h)
       {
           for (int y = (int)*hightest; y >= jump_h-5; y--)
           {
               for (int x = l_border[*hightest+1]; x <= r_border[*hightest+1]; x++)
               {
                   if(image[y][x] == 255)
                   {
                       break;
                   }
                   if(x == r_border[*hightest+1])
                   {
                       blake_line++;
                   }
               }
           }
       }
        if(blake_line >= 3)
        {
            if(l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,70 ,90) == 0
                    && r_loss_judge(r_border,90 ,110) == 0 && r_loss_judge(r_border,70 ,90) == 0
                    && monotonicity_l == 1 && monotonicity_r == 1)
                if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE && sum_island == 0 && island == 0
                        && cross_sum == 0)
                    jump_position = 1;
            if(eresting==1)
            {
                n=*hightest;
                m=(int)now_velocity;
                eresting=0;
            }
        }
    }
//    if(jump_position == 1)
//    {
//
//       if(*hightest < (uint8)jump_h)
//       {
//           jump_position = 0;
//       }
//    }
}

/*·
函数名称：void image_process(void)
功能说明：最终处理函数
参数说明：无
函数返回：无
修改时间：2025年1月5日
备    注：
example： image_process();
 */
void image_process(void)
{
uint8 hightest = 0;//定义一个最高行，tip：这里的最高指的是y值的最小
/*这是离线调试用的*/
Get_image(mt9v03x_image);//获取图像
turn_to_bin();//二值化
/*提取赛道边界*/
image_filter(bin_image);//滤波
image_draw_rectan(bin_image);//预处理
//清零
data_stastics_l = 0;
data_stastics_r = 0;
data_stastics_l_fill = 0;
data_stastics_r_fill = 0;
if (get_start_point(image_h - 2))//找到起点了，再执行八领域
{
    search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);

    // 从爬取的边界线内提取边线 ， 这个才是最终有用的边线
    get_left(data_stastics_l);
    get_right(data_stastics_r);
    //单调性
    monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
    //十字补线函数
    //cross_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
    //跳跃
    jump_judge(bin_image,&hightest,l_border,r_border,monotonicity_l, monotonicity_r,(Encoder_Left-Encoder_Right)/2);
    //斑马线
    cross_stop(bin_image,l_border,r_border);
    //环岛
    around_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index,monotonicity_l, monotonicity_r);
   //处理函数放这里，不要放到if外面去了，不要放到if外面去了，不要放到if外面去了，重要的事说三遍

}
if(get_start_point(image_h - 2) == 0 && jump_position == 0) //除跳跃没找到起点，进入停车程序
{
    target_velocity = 0;
    pit_all_close ();
    small_driver_set_duty(0, 0);
    string_not = 1;
}
//拟合中线
//加入两线判断

//加入两线判断
for (int i = hightest; i < image_h-1; i++)
    {
    //加入两线判断
    x1_boundary[i] = l_border[i];
    x2_boundary[i] = center_line[i];
    x3_boundary[i] = r_border[i];
    //加入两线判断
    center_line[i] = (l_border[i] + r_border[i]) >> 1;//求中线
    //求中线最好最后求，不管是补线还是做状态机，全程最好使用一组边线，中线最后求出，不能干扰最后的输出
    //当然也有多组边线的找法，但是个人感觉很繁琐，不建议
    //加入三线判断

    //加入三线判断
    }
//加入三线判断
//单边桥补线
bridge_fill(bin_image,l_border, r_border, center_line, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index);
//加入三线判断

//蜂鸣器响应
if(jump_position == 0 && stop_position == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
       && cross_sum == 0 && island == 0 && sum_island == 0)
{
    buzzer = 0;
}
else
    buzzer = 1;
//中线取点
border = (float)((center_line[image_h-2] * 0.07) + (center_line[image_h-12] * 0.10)
       + (center_line[image_h-22] * 0.15) + (center_line[image_h-27] * 0.25)
       + (center_line[image_h-32] * 0.17) + (center_line[image_h-42] * 0.09)
       + (center_line[image_h-52] * 0.07) + (center_line[image_h-62] * 0.06)
       + (center_line[image_h-72] * 0.04));
//参数设置
s=stop_position;
u=cross_sum;
c=monotonicity_l;
e=monotonicity_r;
v=island;
f=sum_island;
q=BridgeState;
j=jump_position;
//显示图像
//tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);//显示原图像
//tft180_show_gray_image (0,60, bin_image[0], image_w, image_h, image_w/2, image_h/2,0);//显示二值图
//    for (int i = 0; i+1 < data_stastics_l; i++)
//    {
//        //tft180_draw_point((points_l[i][0]+2)/2, points_l[i][1]/2, RGB565_BLUE);//显示左线
//        tft180_draw_line((points_l[i][0]+2)/2, points_l[i][1]/2, (points_l[i+1][0]+2)/2, points_l[i+1][1]/2,RGB565_BLUE);
//    }
//    for (int i = 0; i+1 < data_stastics_r; i++)
//    {
//        //tft180_draw_point((points_r[i][0]-2)/2, points_r[i][1]/2, RGB565_RED);//显示右线
//        tft180_draw_line((points_r[i][0]-2)/2, points_r[i][1]/2, (points_r[i+1][0]-2)/2, points_r[i+1][1]/2,RGB565_RED);
//    }
//    for (int i = 0; i < image_h-1; i++)
//        {
//            tft180_draw_point(center_line[i]/2, i/2, RGB565_GREEN);// 显示中线
//            tft180_draw_point(l_border[i]/2, i/2, RGB565_GREEN);//显示起点 显示左边线
//            tft180_draw_point(r_border[i]/2, i/2, RGB565_GREEN);//显示起点 显示右边线
//        }
}
