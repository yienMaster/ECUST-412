#ifndef __ELEMENT_H__
#define __ELEMENT_H__

#include "zf_common_headfile.h"

//环岛状态机
typedef enum
{
    ROUND_READY,//0
    GET_IN_READY,//1
    GET_IN_ROUND,//2
    IN_ROUND,//3
    OUTTING,//4
    GET_OUT_ROUND,//5
    OUT_ROUND,//6
}Round_FSM_enum;//环岛

typedef enum
{
    ROUND_L,//0
    ROUND_R,//1
    NO_ROUND,//2
}Round_enum;

//元素种类
typedef enum
{
    IDLE,           //初始
    STRAIGHT,       //直道
    CURVE,          //弯道
    CROSSING,       //十字
    ROUND,          //环岛
    ZEBRA           //斑马线
}Element_FSM_enum;


typedef struct {
        uint8 row;
        uint8 col;
}Point_Structure;//角点



typedef enum
{
    STRONG_FOUR,                            //四个强角点
    STRONG_UP_TWO,                          //两个上强角点
    STRONG_DOWN_TWO,                        //两个下强角点
    STRONG_LEFT_UP_WEAK_LEFT,               //左上强下弱
    STRONG_LEFT_UP_STRONG_LEFT_DOWN,        //左上强下强
    STRONG_UP_STRONG_LEFT_DOWN,             //两个上强角点一个左下强角点
    STRONG_UP_WEAK_LEFT,                    //两个上强角点一个左弱角点
    STRONG_RIGHT_UP_WEAK_RIGHT,             //右上强下弱
    STRONG_RIGHT_UP_STRONG_RIGHT_DOWN,      //右上强下强
    STRONG_UP_STRONG_RIGHT_DOWN,            //两个上强角点一个右下强角点
    STRONG_UP_WEAK_RIGHT,                   //两个上强角点一个右弱角点
    STRONG_LEFT_WEAK_RIGHT,                 //两个左强角点一个右弱角点
    STRONG_RIGHT_WEAK_LEFT,                 //两个右强角点一个左弱角点
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



#define ANGLE_CONSTANT_LITTLE           (3)         //角点判断有边线处阈值
#define ANGLE_CONSTANT_LARGE            (10)         //角点判断无边线处阈值



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
