#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "zf_common_headfile.h"

//����״̬��
typedef enum
{
    ROUND_READY,//0
    GET_IN_READY,//1
    GET_IN_ROUND,//2
    IN_ROUND,//3
    OUTTING,//4
    GET_OUT_ROUND,//5
    OUT_ROUND,//6
}Round_FSM_enum;//����

typedef enum
{
    ROUND_L,//0
    ROUND_R,//1
    NO_ROUND,//2
}Round_enum;

//Ԫ������
typedef enum
{
    IDLE,           //��ʼ
    STRAIGHT,       //ֱ��
    CURVE,          //���
    CROSSING,       //ʮ��
    ROUND,          //����
    ZEBRA           //������
}Element_FSM_enum;


typedef struct {
        uint8 row;
        uint8 col;
}Point_Structure;//�ǵ�



typedef enum
{
    STRONG_FOUR,                            //�ĸ�ǿ�ǵ�
    STRONG_UP_TWO,                          //������ǿ�ǵ�
    STRONG_DOWN_TWO,                        //������ǿ�ǵ�
    STRONG_LEFT_UP_WEAK_LEFT,               //����ǿ����
    STRONG_LEFT_UP_STRONG_LEFT_DOWN,        //����ǿ��ǿ
    STRONG_UP_STRONG_LEFT_DOWN,             //������ǿ�ǵ�һ������ǿ�ǵ�
    STRONG_UP_WEAK_LEFT,                    //������ǿ�ǵ�һ�������ǵ�
    STRONG_RIGHT_UP_WEAK_RIGHT,             //����ǿ����
    STRONG_RIGHT_UP_STRONG_RIGHT_DOWN,      //����ǿ��ǿ
    STRONG_UP_STRONG_RIGHT_DOWN,            //������ǿ�ǵ�һ������ǿ�ǵ�
    STRONG_UP_WEAK_RIGHT,                   //������ǿ�ǵ�һ�������ǵ�
    STRONG_LEFT_WEAK_RIGHT,                 //������ǿ�ǵ�һ�������ǵ�
    STRONG_RIGHT_WEAK_LEFT,                 //������ǿ�ǵ�һ�������ǵ�
    NO_CROSS
}Crossing_enum;

typedef enum
{
    CURVE_L,
    CURVE_R,
    NO_CURVE
}Curve_enum;

typedef enum
{
    ZEBRA_START,
    ZEBRA_FIND,
    ZEBRA_MAINTAIN,
    ZEBRA_STOP
}Zebra_FSM_enum;



#define ANGLE_CONSTANT_LITTLE           (3)         //�ǵ��ж��б��ߴ���ֵ
#define ANGLE_CONSTANT_LARGE            (10)         //�ǵ��ж��ޱ��ߴ���ֵ



uint8 Find_Strong_Angle_Left_Up(uint8 start_row,uint8 end_row);
uint8 Find_Strong_Angle_Left_Down(uint8 start_row,uint8 end_row);
uint8 Find_Strong_Angle_Right_Up(uint8 start_row,uint8 end_row);
uint8 Find_Strong_Angle_Right_Down(uint8 start_row,uint8 end_row);
uint8 Find_Weak_Angle_Left(uint8 start_row,uint8 end_row);
uint8 Find_Weak_Angle_Right(uint8 start_row,uint8 end_row);
void Add_Line(uint8 position,uint8 length,uint8 row_up,uint8 col_up,uint8 row_down,uint8 col_down);
void Add_Line_Left_By_Two_points(Point_Structure up_point,Point_Structure down_point);
void Add_Line_Right_By_Two_points(Point_Structure up_point,Point_Structure down_point);
void Add_Line_Left_By_One_point_Up(Point_Structure point);
void Add_Line_Right_By_One_point_Up(Point_Structure point);
void Add_Line_Left_By_One_point_down(Point_Structure point);
void Add_Line_Right_By_One_point_down(Point_Structure point);
void Add_Line_Left_By_Round_Point(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2);
void Add_Line_Right_By_Round_Point(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2);
uint8 Find_Left_Round_Point(uint8 start_row,uint8 end_row);
uint8 Find_Right_Round_Point(uint8 start_row,uint8 end_row);
uint8 Calculate_Linear_Degree(int16* line);
void Outside_Bound_Judge(void);
Crossing_enum Crossing_Judge(void);
void Crossing_Add_Line(void);
Curve_enum Curve_Judge(void);
Round_enum Round_Judge(void);
void Round_State_Update(void);
void Element_Judge(void);
void Element_Deal(void);


extern Point_Structure strong_angle_left_up,strong_angle_left_down,strong_angle_right_up,strong_angle_right_down;
extern Point_Structure weak_angle_left,weak_angle_right;
extern Element_FSM_enum element_state;
extern Round_FSM_enum round_state;
extern Zebra_FSM_enum zebra_state;
extern Round_enum round_position;
extern Curve_enum curve_position;
extern Crossing_enum crossing_position;
extern Point_Structure round_point_left,round_point_right;
extern uint8 zebra_time_flag;
extern uint8 zebra_time_count;










#endif
