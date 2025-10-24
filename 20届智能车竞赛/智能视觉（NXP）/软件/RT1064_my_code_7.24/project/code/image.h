#ifndef _IMAGE_H_
#define _IMAGE_H_
#include "zf_common_headfile.h"

//数据类型声明
//typedef   signed          char int8;
//typedef   signed short     int int16;
//typedef   signed           int int32;
//typedef unsigned          char uint8;
//typedef unsigned short     int uint16;
//typedef unsigned           int uint32;

//颜色定义  因为有先例，连颜色都改不来，我直接放这了
#define my_RED     0XF800    //红色
#define my_GREEN   0X07E0    //绿色
#define my_BLUE    0X001F    //蓝色

//宏定义
#define image_h	120//图像高度
#define image_w	188//图像宽度

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//跳过的点数
#define border_max	image_w-2 //边界最大值
#define border_min	1	//边界最小值	

#define USE_num	image_h*3	//定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点

//导出的变量
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
extern uint8 l_border[image_h];//左线数组           下标是y(行号),内容是x(列号)
extern uint8 r_border[image_h];//右线数组           下标是y(行号),内容是x(列号)
extern uint16 points_l[(uint16)USE_num][2];//左线
extern uint16 points_r[(uint16)USE_num][2];//右线
extern uint8 center_line[image_h];
extern uint16 search_times;;
extern bool zebra_detect;
extern uint16 l_index[image_h];//映射左线数组(左线数组从边界数组中取出的点的下表
                               //如:r_border[h]=points_r[j][0]-1;
                               //   r_index[h] = j; 记录下j
extern uint16 r_index[image_h];//映射右线数组
//函数定义
int my_abs(int value);
int limit_a_b(int x, int min, int max);

void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了

#endif

