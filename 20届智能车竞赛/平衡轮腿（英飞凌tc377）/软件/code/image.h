#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"
#include "control.h"

//宏定义
#define image_h 120//图像高度
#define image_w 188//图像宽度


#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//跳过的点数
#define border_max  image_w-2 //边界最大值
#define border_min  1   //边界最小值
#define USE_num image_h*3   //定义找点的数组成员个数按理说300个点能放下，但是有些特殊情况确实难顶，多定义了一点


typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // 不在单边桥状态
    SINGLE_BRIDGE_ACTIVE       // 在单边桥状态
} SingleBridgeState;

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//图像数组
// 只有X边界
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
extern uint16 points_l[(uint16)USE_num][2];//左线
extern uint16 points_r[(uint16)USE_num][2];//右线
extern uint16 data_stastics_l;//统计左边找到点的个数
extern uint16 data_stastics_r;//统计右边找到点的个数


extern float border;//中线拟合标志位
extern SingleBridgeState BridgeState;//单边桥标志位
extern int jump_position;//跳跃标志位
extern int stop_position;//斑马线标志位
extern int buzzer;//蜂鸣器标志位
extern int string_not;//出线标志位jump_h
extern int jump_h;//跳跃高度
extern int stop;
//参数
extern int m;
extern int n;
extern int s;
extern int v;
extern int c;
extern int e;
extern int f;
extern int q;
extern int j;
extern int u;

extern void image_process(void); //直接在中断或循环里调用此程序就可以循环执行了

#endif /*_IMAGE_H*/

