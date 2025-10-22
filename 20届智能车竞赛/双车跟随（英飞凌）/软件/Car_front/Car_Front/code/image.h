#ifndef __IMAGE_H__
#define __IMAGE_H__
#include "zf_common_headfile.h"



typedef struct
{
        uint8 col;          //�к�
        uint8 highest_row;  //����к�
}Longest_Col_Structure;     //����нṹ��

//�궨��
#define GrayScale               (256)
#define WHITE_IMG               (255)
#define BLACK_IMG               (0)
#define RED_IMG                 (128)
#define BLUE_IMG                (200)
#define GREEN_IMG               (72)

#define LINE_INVALID_LEFT       (-254)      //������Чֵ
#define LINE_INVALID_RIGHT      (254)       //������Чֵ
#define LINE_INVALID_MIDDLE     (254)       //������Чֵ


//��������*************
//ͼ���ʼ�ı���
extern Longest_Col_Structure longest_col_left,longest_col_right;
extern int16 line_left[MT9V03X_H],line_right[MT9V03X_H],line_middle[MT9V03X_H];//
extern uint8 img_data[MT9V03X_H][MT9V03X_W];//ԭʼͼ��
extern uint8 binary_img_data[MT9V03X_H][MT9V03X_W];//��ֵ��ͼ��
extern uint8 ostu_threshold;
extern uint8 white_col_mid_row;
//��������


//������
uint8 Otsu_Threshold(uint8 *image, uint16 col, uint16 row);
void IMG_Init(void);
void Line_Init(void);
void Longest_Col(void);
void Left_Line(void);
void Right_Line(void);
void Middle_Line(void);
void Image_Deal(void);


#endif
