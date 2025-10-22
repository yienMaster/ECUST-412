#ifndef __IMAGE_H__
#define __IMAGE_H__
#include "zf_common_headfile.h"



typedef struct
{
        uint8 col;          //列号
        uint8 highest_row;  //最高行号
}Longest_Col_Structure;     //最长白列结构体

//宏定义
#define GrayScale               (256)
#define WHITE_IMG               (255)
#define BLACK_IMG               (0)
#define RED_IMG                 (128)
#define BLUE_IMG                (200)
#define GREEN_IMG               (72)

#define LINE_INVALID_LEFT       (-254)      //左线无效值
#define LINE_INVALID_RIGHT      (254)       //右线无效值
#define LINE_INVALID_MIDDLE     (254)       //中线无效值


//变量引用*************
//图像初始的变量
extern Longest_Col_Structure longest_col_left,longest_col_right;
extern int16 line_left[MT9V03X_H],line_right[MT9V03X_H],line_middle[MT9V03X_H];//
extern uint8 img_data[MT9V03X_H][MT9V03X_W];//原始图像
extern uint8 binary_img_data[MT9V03X_H][MT9V03X_W];//二值化图像
extern uint8 ostu_threshold;
extern uint8 white_col_mid_row;
//环岛变量


//函数区
uint8 Otsu_Threshold(uint8 *image, uint16 col, uint16 row);
void IMG_Init(void);
void Line_Init(void);
void Longest_Col(void);
void Left_Line(void);
void Right_Line(void);
void Middle_Line(void);
void Image_Deal(void);


#endif
