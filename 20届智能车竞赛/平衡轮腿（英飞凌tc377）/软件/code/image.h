#ifndef _IMAGE_H
#define _IMAGE_H
#include "zf_common_headfile.h"
#include "control.h"

//�궨��
#define image_h 120//ͼ��߶�
#define image_w 188//ͼ����


#define white_pixel 255
#define black_pixel 0

#define bin_jump_num    1//�����ĵ���
#define border_max  image_w-2 //�߽����ֵ
#define border_min  1   //�߽���Сֵ
#define USE_num image_h*3   //�����ҵ�������Ա��������˵300�����ܷ��£�������Щ�������ȷʵ�Ѷ����ඨ����һ��


typedef enum {
    SINGLE_BRIDGE_NOT_ACTIVE,  // ���ڵ�����״̬
    SINGLE_BRIDGE_ACTIVE       // �ڵ�����״̬
} SingleBridgeState;

extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//ͼ������
// ֻ��X�߽�
extern uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];
extern uint16 points_l[(uint16)USE_num][2];//����
extern uint16 points_r[(uint16)USE_num][2];//����
extern uint16 data_stastics_l;//ͳ������ҵ���ĸ���
extern uint16 data_stastics_r;//ͳ���ұ��ҵ���ĸ���


extern float border;//������ϱ�־λ
extern SingleBridgeState BridgeState;//�����ű�־λ
extern int jump_position;//��Ծ��־λ
extern int stop_position;//�����߱�־λ
extern int buzzer;//��������־λ
extern int string_not;//���߱�־λjump_h
extern int jump_h;//��Ծ�߶�
extern int stop;
//����
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

extern void image_process(void); //ֱ�����жϻ�ѭ������ô˳���Ϳ���ѭ��ִ����

#endif /*_IMAGE_H*/

