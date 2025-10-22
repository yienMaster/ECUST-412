#include "element.h"
#include "zf_common_headfile.h"



Point_Structure strong_angle_left_up,strong_angle_left_down,strong_angle_right_up,strong_angle_right_down;  //强角点
Point_Structure weak_angle_left,weak_angle_right;   //弱角点
Point_Structure round_point_left,round_point_right; //环上的点
Element_FSM_enum element_state=IDLE; //元素状态机标志
Round_FSM_enum round_state; //环岛状态机标志
Round_enum round_position;  //环岛位置
Curve_enum curve_position;  //弯道位置
Crossing_enum crossing_position;    //十字位置
Zebra_FSM_enum zebra_state=ZEBRA_START;
uint8 round_state_update_count=0;
uint8 zebra_state_update_count=0;
float standard_k=0.71;//标准k值
uint8 zebra_row;
uint8 zebra_time_flag=0;
uint8 zebra_time_count=0;
uint8 round_out_point_row,round_out_point_col;



uint8 Find_Strong_Angle_Left_Up(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_left[i-3]!=LINE_INVALID_LEFT
         &&line_left[i-2]!=LINE_INVALID_LEFT
         &&line_left[i-1]!=LINE_INVALID_LEFT
         &&line_left[i]!=LINE_INVALID_LEFT
         &&line_left[i]>20 && line_left[i]<MT9V03X_W-21
         &&(line_left[i-3]-line_left[i-2]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i-2]-line_left[i-1]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i-1]-line_left[i]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i-3]-line_left[i]>=-1)
         &&(line_left[i-2]-line_left[i]>=-1)
         &&(line_left[i-1]-line_left[i]>=-1)
         &&((line_left[i]-line_left[i+1]>ANGLE_CONSTANT_LARGE)
         ||(line_left[i]-line_left[i+2]>ANGLE_CONSTANT_LARGE)
         ||(line_left[i]-line_left[i+3]>ANGLE_CONSTANT_LARGE)))
        {
            strong_angle_left_up.row=i;
            strong_angle_left_up.col=(uint8)line_left[i];
            Record_point(strong_angle_left_up.row,strong_angle_left_up.col,0);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Strong_Angle_Left_Down(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_left[i+3]!=LINE_INVALID_LEFT
         &&line_left[i+2]!=LINE_INVALID_LEFT
         &&line_left[i+1]!=LINE_INVALID_LEFT
         &&line_left[i]!=LINE_INVALID_LEFT
         &&line_left[i]>20 && line_left[i]<MT9V03X_W-21
         &&(line_left[i]-line_left[i+1]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i+1]-line_left[i+2]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i+2]-line_left[i+3]<ANGLE_CONSTANT_LITTLE)
         &&(line_left[i]-line_left[i+1]>=-1)
         &&(line_left[i]-line_left[i+2]>=-1)
         &&(line_left[i]-line_left[i+3]>=-1)
         &&((line_left[i]-line_left[i-1]>ANGLE_CONSTANT_LARGE)
         ||(line_left[i]-line_left[i-2]>ANGLE_CONSTANT_LARGE)
         ||(line_left[i]-line_left[i-3]>ANGLE_CONSTANT_LARGE)))
        {
            strong_angle_left_down.row=i;
            strong_angle_left_down.col=(uint8)line_left[i];
            Record_point(strong_angle_left_down.row,strong_angle_left_down.col,0);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Strong_Angle_Right_Up(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_right[i-3]!=LINE_INVALID_RIGHT
         &&line_right[i-2]!=LINE_INVALID_RIGHT
         &&line_right[i-1]!=LINE_INVALID_RIGHT
         &&line_right[i]!=LINE_INVALID_RIGHT
         &&line_right[i]>20 && line_right[i]<MT9V03X_W-21
         &&(line_right[i-2]-line_right[i-3]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i-1]-line_right[i-2]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i]-line_right[i-1]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i]-line_right[i-3]>=-1)
         &&(line_right[i]-line_right[i-2]>=-1)
         &&(line_right[i]-line_right[i-1]>=-1)
         &&((line_right[i+1]-line_right[i]>ANGLE_CONSTANT_LARGE)
         ||(line_right[i+2]-line_right[i]>ANGLE_CONSTANT_LARGE)
         ||(line_right[i+3]-line_right[i]>ANGLE_CONSTANT_LARGE)))
        {
            strong_angle_right_up.row=i;
            strong_angle_right_up.col=(uint8)line_right[i];
            Record_point(strong_angle_right_up.row,strong_angle_right_up.col,0);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Strong_Angle_Right_Down(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_right[i+3]!=LINE_INVALID_RIGHT
         &&line_right[i+2]!=LINE_INVALID_RIGHT
         &&line_right[i+1]!=LINE_INVALID_RIGHT
         &&line_right[i]!=LINE_INVALID_RIGHT
         &&line_right[i]>20 && line_right[i]<MT9V03X_W-21
         &&(line_right[i+1]-line_right[i]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i+2]-line_right[i+1]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i+3]-line_right[i+2]<ANGLE_CONSTANT_LITTLE)
         &&(line_right[i+1]-line_right[i]>=-1)
         &&(line_right[i+2]-line_right[i]>=-1)
         &&(line_right[i+3]-line_right[i]>=-1)
         &&((line_right[i-1]-line_right[i]>ANGLE_CONSTANT_LARGE)
         ||(line_right[i-2]-line_right[i]>ANGLE_CONSTANT_LARGE)
         ||(line_right[i-3]-line_right[i]>ANGLE_CONSTANT_LARGE)))
        {
            strong_angle_right_down.row=i;
            strong_angle_right_down.col=(uint8)line_right[i];
            Record_point(strong_angle_right_down.row,strong_angle_right_down.col,0);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Weak_Angle_Left(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_left[i-3]!=LINE_INVALID_LEFT
         &&line_left[i-2]!=LINE_INVALID_LEFT
         &&line_left[i-1]!=LINE_INVALID_LEFT
         &&line_left[i]!=LINE_INVALID_LEFT
         &&line_left[i+1]!=LINE_INVALID_LEFT
         &&line_left[i+2]!=LINE_INVALID_LEFT
         &&line_left[i+3]!=LINE_INVALID_LEFT
         &&line_left[i]>4 && line_left[i]<MT9V03X_W-5
         &&(line_left[i-3]-line_left[i-2]<0)
         &&(line_left[i-2]-line_left[i-1]<=0)
         &&(line_left[i-1]-line_left[i]<=0)
         &&(line_left[i]-line_left[i+1]>=0)
         &&(line_left[i+1]-line_left[i+2]>=0)
         &&(line_left[i+2]-line_left[i+3]>0))
        {
            weak_angle_left.row=i;
            weak_angle_left.col=(uint8)line_left[i];
            Record_point(weak_angle_left.row,weak_angle_left.col,1);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Weak_Angle_Right(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_right[i-3]!=LINE_INVALID_RIGHT
         &&line_right[i-2]!=LINE_INVALID_RIGHT
         &&line_right[i-1]!=LINE_INVALID_RIGHT
         &&line_right[i]!=LINE_INVALID_RIGHT
         &&line_right[i]>4 && line_right[i]<MT9V03X_W-5
         &&line_right[i+1]!=LINE_INVALID_RIGHT
         &&line_right[i+2]!=LINE_INVALID_RIGHT
         &&line_right[i+3]!=LINE_INVALID_RIGHT
         &&(line_right[i-3]-line_right[i-2]>0)
         &&(line_right[i-2]-line_right[i-1]>=0)
         &&(line_right[i-1]-line_right[i]>=0)
         &&(line_right[i]-line_right[i+1]<=0)
         &&(line_right[i+1]-line_right[i+2]<=0)
         &&(line_right[i+2]-line_right[i+3]<0))
        {
            weak_angle_right.row=i;
            weak_angle_right.col=(uint8)line_right[i];
            Record_point(weak_angle_right.row,weak_angle_right.col,1);
            return 1;
        }
    }
    return 0;
}
uint8 Find_Left_Round_Point(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_left[i]!=LINE_INVALID_LEFT
           &&line_left[i]>=line_left[i-1]
           &&line_left[i-1]>=line_left[i-2]
           &&line_left[i-2]>=line_left[i-3]
           &&line_left[i]>=line_left[i+1]
           &&line_left[i+1]>=line_left[i+2]
           &&line_left[i+2]>=line_left[i+3]
           &&line_left[i]>20
           &&line_left[i]<MT9V03X_W-20)
        {
            round_point_left.row=i;
            round_point_left.col=(uint8)line_left[i];
            Record_point(round_point_left.row,round_point_left.col,1);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Right_Round_Point(uint8 start_row,uint8 end_row)
{
    uint8 i=0;
    for(i=start_row;i<end_row;i++)
    {
        if(line_right[i]!=LINE_INVALID_RIGHT
           &&line_right[i]<=line_right[i-1]
           &&line_right[i-1]<=line_right[i-2]
           &&line_right[i-2]<=line_right[i-3]
           &&line_right[i]<=line_right[i+1]
           &&line_right[i+1]<=line_right[i+2]
           &&line_right[i+2]<=line_right[i+3]
           &&line_left[i]>20
           &&line_left[i]<MT9V03X_W-20)
        {
            round_point_right.row=i;
            round_point_right.col=(uint8)line_right[i];
            Record_point(round_point_right.row,round_point_right.col,1);
            return 1;
        }
    }
    return 0;
}



void Add_Line(uint8 position,uint8 length,uint8 row_up,uint8 col_up,uint8 row_down,uint8 col_down)
{
    float k,b;
    uint8 i;
    if(position==0) //连左线
    {
        k=Calculate_Slope(row_up,col_up,row_down,col_down);
        b=col_up-k*row_up;
        if(length==0)   //连接两点之间
        {
            for(i=row_up+1;i<row_down;i++)
            {
                line_left[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
        else if(length==1)    //向下延长
        {
            for(i=row_up+1;i<MT9V03X_H-1;i++)
            {
                line_left[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
        else        //向上延长
        {
            for(i=0;i<row_down;i++)
            {
                line_left[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }

    }
    else    //连右线
    {
        k=Calculate_Slope(row_up,col_up,row_down,col_down);
        b=col_up-k*row_up;
        if(length==0)   //连接两点之间
        {
            for(i=row_up+1;i<row_down;i++)
            {
                line_right[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
        else if(length==1)   //向下延长
        {
            for(i=row_up+1;i<MT9V03X_H-1;i++)
            {
                line_right[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
        else
        {
            for(i=0;i<row_down;i++)
            {
                line_right[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
    }

}




void Add_Line_Left_Round_Up_Point(Point_Structure point)//左环岛状态2补线
{
    float k,b;
    uint8 i,j;
    uint8 white_count,row_start,white_count_max=0;
    for(i=3;i<point.col-3;i++)
    {
        white_count=0; row_start=MT9V03X_H-1;
        for(j=MT9V03X_H-1;j>0;j--)
        {
            if(binary_img_data[j][i]==WHITE_IMG)
            {
                white_count++;
                if(white_count==1)  //第一个白块对应行为起始行
                    row_start=j;
            }
            if(white_count!=0 && binary_img_data[j][i]==BLACK_IMG)  //找到黑块
                break;
            if(j<MT9V03X_H>>1 && white_count==0)    //下方大量黑块
                break;
        }
        if(white_count>white_count_max) //更新最长白列
        {
            white_count_max=white_count;
            longest_col_left.col=i;
            longest_col_left.highest_row=row_start-white_count+1;
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
        else if(white_count==white_count_max)   //更新右列
        {
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
    }
    i=MT9V03X_H-20;
    if(line_right[i]!=LINE_INVALID_RIGHT)
    {
        k=Calculate_Slope(point.row,point.col,i,(uint8)line_right[i]);
        b=point.col-k*point.row;
    }
    else
    {
        k=Calculate_Slope(point.row,point.col,MT9V03X_H-20,MT9V03X_W-1);
        b=point.col-k*point.row;
    }
    for(i=0;i<MT9V03X_H;i++)
    {
        if(i<=point.row)
            line_left[i]=LINE_INVALID_LEFT;
        line_right[i]=LINE_INVALID_RIGHT;
    }
    for(i=point.row;i<MT9V03X_H;i++)
    {
        line_right[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
    }
    for(i=point.row;i>Max(longest_col_left.highest_row,longest_col_right.highest_row);i--)
    {
        for(j=point.col;j>0;j--)
        {
          if(binary_img_data[i][j] == WHITE_IMG && binary_img_data[i][j + 1] == WHITE_IMG)
            {
                if(binary_img_data[i][j + 2] == BLACK_IMG && binary_img_data[i][j + 3] == BLACK_IMG)
                {
                    line_right[i]=j+2;
                    break;
                }

            }
        }
    }
}

void Add_Line_Right_Round_Up_Point(Point_Structure point)//右环岛状态2补线
{
    float k,b;
    uint8 i,j;
    uint8 white_count,row_start,white_count_max=0;
    for(i=point.col+3;i<MT9V03X_W-3;i++)
    {
        white_count=0; row_start=MT9V03X_H-1;
        for(j=MT9V03X_H-1;j>0;j--)
        {
            if(binary_img_data[j][i]==WHITE_IMG)
            {
                white_count++;
                if(white_count==1)  //第一个白块对应行为起始行
                    row_start=j;
            }
            if(white_count!=0 && binary_img_data[j][i]==BLACK_IMG)  //找到黑块
                break;
            if(j<MT9V03X_H>>1 && white_count==0)    //下方大量黑块
                break;
        }
        if(white_count>white_count_max) //更新最长白列
        {
            white_count_max=white_count;
            longest_col_left.col=i;
            longest_col_left.highest_row=row_start-white_count+1;
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
        else if(white_count==white_count_max)   //更新右列
        {
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
    }
    i=MT9V03X_H-20;
    if(line_left[i]!=LINE_INVALID_LEFT)
    {
        k=Calculate_Slope(point.row,point.col,i,(uint8)line_left[i]);
        b=point.col-k*point.row;
    }
    else
    {
        k=Calculate_Slope(point.row,point.col,MT9V03X_H-20,0);
        b=point.col-k*point.row;
    }
    for(i=0;i<MT9V03X_H;i++)
    {
        line_left[i]=LINE_INVALID_LEFT;
        if(i<=point.row)
            line_right[i]=LINE_INVALID_RIGHT;
    }
    for(i=point.row;i<MT9V03X_H;i++)
    {
        line_left[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
    }
    for(i=point.row;i>Max(longest_col_left.highest_row,longest_col_right.highest_row);i--)
    {
        for(j=MT9V03X_W;j>point.col;j--)
        {
            if(binary_img_data[i][j] == WHITE_IMG && binary_img_data[i][j - 1] == WHITE_IMG)
            {
                if(binary_img_data[i][j - 2] == BLACK_IMG && binary_img_data[i][j - 3] == BLACK_IMG)
                {
                    line_left[i]=j-2;
                    break;
                   }

               }
        }
    }
}

void Add_Line_Left_By_Round_Point(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2)
{
    float k1,k2,b;
    double angular_value;//夹角绝对值
    uint8 i;
    k1=Calculate_Slope(row_1,col_1,row_2,col_2);
    angular_value=atan(k1)-atan(standard_k);
    k2=(float)tan(atan(-standard_k)+angular_value);
    b=round_point_left.col-k2*round_point_left.row;
    for(i=0;i<MT9V03X_H;i++)
    {
        line_left[i]=Constrain_int16((int16)(k2*i+b),MT9V03X_W-1,0);
    }
}


void Add_Line_Right_By_Round_Point(uint8 row_1,uint8 col_1,uint8 row_2,uint8 col_2)
{
    float k1,k2,b;
    double angular_value;//夹角值
    uint8 i;
    k1=Calculate_Slope(row_1,col_1,row_2,col_2);
    angular_value=atan(k1)-atan(-standard_k);
    k2=(float)tan(atan(standard_k)+angular_value);
    b=round_point_right.col-k2*round_point_right.row;
    for(i=0;i<MT9V03X_H;i++)
    {
        line_right[i]=Constrain_int16((int16)(k2*i+b),MT9V03X_W-1,0);
    }
}

void Add_Line_Zebra(uint8 start_row,uint8 end_row)
{
    float k1,k2,b1,b2;
    uint8 i;
    if(line_left[start_row]>=45&&line_right[start_row]<=135)
    {
        if(start_row==zebra_row)
        {
            k1=Calculate_Slope(start_row+10,(uint8)line_left[start_row+10],end_row,(uint8)line_left[end_row]);
            k2=Calculate_Slope(start_row+10,(uint8)line_right[start_row+10],end_row,(uint8)line_right[end_row]);
            b1=line_left[end_row]-k1*end_row;
            b2=line_right[end_row]-k2*end_row;
        }
        else if(zebra_row+30>=end_row)
        {
            k1=Calculate_Slope(start_row,(uint8)line_left[start_row],start_row+10,(uint8)line_left[start_row+10]);
            k2=Calculate_Slope(start_row,(uint8)line_right[start_row],start_row+10,(uint8)line_right[start_row+10]);
            b1=line_left[start_row]-k1*start_row;
            b2=line_right[start_row]-k2*start_row;
        }
        else
        {
            k1=Calculate_Slope(start_row,(uint8)line_left[start_row],end_row,(uint8)line_left[end_row]);
            k2=Calculate_Slope(start_row,(uint8)line_right[start_row],end_row,(uint8)line_right[end_row]);
            b1=line_left[start_row]-k1*start_row;
            b2=line_right[start_row]-k2*start_row;
        }
        for(i=start_row;i<end_row;i++)
        {
            line_left[i]=Constrain_int16((int16)(k1*i+b1),MT9V03X_W-1,0);
            line_right[i]=Constrain_int16((int16)(k2*i+b2),MT9V03X_W-1,0);
        }
    }
    else
    {
        if(zebra_row+30>=end_row)
        {
//            k1=Calculate_Slope(end_row-60,(uint8)line_left[end_row-60],end_row-50,(uint8)line_left[end_row-50]);
//            k2=Calculate_Slope(end_row-60,(uint8)line_right[end_row-60],end_row-50,(uint8)line_right[end_row-50]);
            k1=-standard_k;
            k2=standard_k;
            b1=line_left[end_row-50]-k1*(end_row-50);
            b2=line_right[end_row-50]-k2*(end_row-50);
        }
        else
        {
            k1=Calculate_Slope(end_row-5,(uint8)line_left[end_row-5],end_row,(uint8)line_left[end_row]);
            k2=Calculate_Slope(end_row-5,(uint8)line_right[end_row-5],end_row,(uint8)line_right[end_row]);
            b1=line_left[end_row]-k1*end_row;
            b2=line_right[end_row]-k2*end_row;
        }
        for(i=end_row-50;i<end_row;i++)
        {
            line_left[i]=Constrain_int16((int16)(k1*i+b1),MT9V03X_W-1,0);
            line_right[i]=Constrain_int16((int16)(k2*i+b2),MT9V03X_W-1,0);
        }

    }

}

void Boundary_Left_Init(void)
{
    uint8 i;
    for(i=0;i<MT9V03X_H;i++)
    {
        line_left[i]=LINE_INVALID_LEFT;
    }
}

void Boundary_Right_Init(void)
{
    uint8 i;
    for(i=0;i<MT9V03X_H;i++)
    {
        line_right[i]=LINE_INVALID_RIGHT;
    }
}


uint8 Calculate_Linear_Degree(int16* line)
{
    float k,b,m,d;
    uint8 i,j,t;
    for(i=MT9V03X_H-1;i>0;i--)
    {
        if(line[i]!=LINE_INVALID_LEFT && line[i]!=LINE_INVALID_RIGHT)
            break;
    }
    if(i<MT9V03X_H-20) return 0;
    for(j=0;j<MT9V03X_H;j++)
    {
        if(line[j]!=LINE_INVALID_LEFT && line[j]!=LINE_INVALID_RIGHT)
            break;
    }
    k=(float)(line[j]-line[i])/(j-i);
    b=line[j]-k*2;
    m=sqrt(k*k+1);
    for(t=j+1;t<i;t++)
    {
        d=Abs(k*t-line[t]+b)/m;
        if(d>2) return 0;
    }
    return 1;
//    float sum_x=0,sum_y=0,sum_xx=0,sum_xy=0,y_mean=0,ss_res=0,ss_tot=0,R2=0;
//    float k=0,b=0;
//    uint8 count=0;
//    uint8 i,temp=0;
//    for(i=MT9V03X_H-1;i>0;i--)
//    {
//        if(line[i]!=LINE_INVALID_LEFT && line[i]!=LINE_INVALID_RIGHT)
//        {
//            temp=i;
//            break;
//        }
//    }
//    for(i=temp;i>0;i--)
//    {
//        if(line[i]!=LINE_INVALID_LEFT && line[i]!=LINE_INVALID_RIGHT)
//        {
//            sum_x += i;
//            sum_y += line[i];
//            sum_xx += i*i;
//            sum_xy += i*line[i];
//            count++;
//        }
//    }
//    k=(count*sum_xy-sum_x*sum_y)/(count*sum_xx-sum_x*sum_x);
//    b=(sum_y-k*sum_x)/count;
//    y_mean=sum_y/count;
//    for(i=temp;i>0;i--)
//    {
//        if(line[i]!=LINE_INVALID_LEFT && line[i]!=LINE_INVALID_RIGHT)
//        {
//            ss_res+=(line[i]-(k*i+b))*(line[i]-(k*i+b));
//            ss_tot+=(line[i]-y_mean)*(line[i]-y_mean);
//        }
//    }
//    R2=1-ss_res/ss_tot;
//    if(line==line_left) R2_left_display=R2;
//    else if(line==line_right) R2_right_display=R2;
//    if(R2>0.985) return 1;
//    else return 0;
}

void Outside_Bound_Judge(void)
{
    uint8 i,j;
    float black_count=0,pixel_count=3*MT9V03X_W;
    for(i=MT9V03X_H-1;i>=MT9V03X_H-3;i--)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(binary_img_data[i][j]==BLACK_IMG)
            {
                black_count++;
            }
        }
    }
    if(black_count/pixel_count>0.85)
    {
        element_state=ZEBRA;
        round_state=NO_ROUND;
        zebra_state=ZEBRA_STOP;
    }
}

Crossing_enum Crossing_Judge(void)
{
    uint8 strong_left_up_flag=Find_Strong_Angle_Left_Up(5,MT9V03X_H-11);
    uint8 strong_right_up_flag=Find_Strong_Angle_Right_Up(5,MT9V03X_H-11);
    uint8 strong_left_down_flag=0,strong_right_down_flag=0,weak_left_flag=0,weak_right_flag=0;
    if(strong_left_up_flag)
    {
        strong_left_down_flag=Find_Strong_Angle_Left_Down(strong_angle_left_up.row+5,MT9V03X_H-6);
        weak_left_flag=Find_Weak_Angle_Left(strong_angle_left_up.row+5,MT9V03X_H-6);
    }
    else
    {
        strong_left_down_flag=Find_Strong_Angle_Left_Down(5,MT9V03X_H-6);
        weak_left_flag=Find_Weak_Angle_Left(5,MT9V03X_H-6);
    }
    if(strong_right_up_flag)
    {
        strong_right_down_flag=Find_Strong_Angle_Right_Down(strong_angle_right_up.row+5,MT9V03X_H-6);
        weak_right_flag=Find_Weak_Angle_Right(strong_angle_right_up.row+5,MT9V03X_H-6);
    }
    else
    {
        strong_right_down_flag=Find_Strong_Angle_Right_Down(5,MT9V03X_H-6);
        weak_right_flag=Find_Weak_Angle_Right(5,MT9V03X_H-6);
    }
    if(strong_left_up_flag && strong_right_up_flag && strong_left_down_flag && strong_right_down_flag)
        return STRONG_FOUR;
    else if(strong_left_up_flag && strong_right_up_flag && strong_left_down_flag)
        return STRONG_UP_STRONG_LEFT_DOWN;
    else if(strong_left_up_flag && strong_right_up_flag && strong_right_down_flag)
        return STRONG_UP_STRONG_RIGHT_DOWN;
    else if(strong_left_up_flag && strong_right_up_flag && weak_left_flag)
        return STRONG_UP_WEAK_LEFT;
    else if(strong_left_up_flag && strong_right_up_flag && weak_right_flag)
        return STRONG_UP_WEAK_RIGHT;
    else if(strong_left_up_flag && strong_left_down_flag && weak_right_flag)
        return STRONG_LEFT_WEAK_RIGHT;
    else if(strong_right_up_flag && strong_right_down_flag && weak_left_flag)
        return STRONG_RIGHT_WEAK_LEFT;
    else if(strong_left_up_flag && strong_right_up_flag)
        return STRONG_UP_TWO;
    else if(strong_left_down_flag && strong_right_down_flag)
        return STRONG_DOWN_TWO;
    else if(strong_left_up_flag && strong_left_down_flag)
        return STRONG_LEFT_UP_STRONG_LEFT_DOWN;
    else if(strong_right_up_flag && strong_right_down_flag)
        return STRONG_RIGHT_UP_STRONG_RIGHT_DOWN;
    else if(strong_left_up_flag && weak_left_flag)
        return STRONG_LEFT_UP_WEAK_LEFT;
    else if(strong_right_up_flag && weak_right_flag)
        return STRONG_RIGHT_UP_WEAK_RIGHT;
    else
        return NO_CROSS;
}

void Crossing_Add_Line(void)
{
    switch(crossing_position)
    {
        case STRONG_FOUR:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,strong_angle_left_down.row,strong_angle_left_down.col);
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,strong_angle_right_down.row,strong_angle_right_down.col);
            break;
        case STRONG_UP_STRONG_LEFT_DOWN:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,strong_angle_left_down.row,strong_angle_left_down.col);
            Add_Line(1,1,strong_angle_right_up.row-5,(uint8)line_right[strong_angle_right_up.row-5],strong_angle_right_up.row,strong_angle_right_up.col);
            break;
        case STRONG_UP_STRONG_RIGHT_DOWN:
            Add_Line(0,1,strong_angle_left_up.row-5,(uint8)line_left[strong_angle_left_up.row-5],strong_angle_left_up.row,strong_angle_left_up.col);
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,strong_angle_right_down.row,strong_angle_right_down.col);
            break;
        case STRONG_UP_WEAK_LEFT:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,weak_angle_left.row,weak_angle_left.col);
            Add_Line(1,1,strong_angle_right_up.row-5,(uint8)line_right[strong_angle_right_up.row-5],strong_angle_right_up.row,strong_angle_right_up.col);
            break;
        case STRONG_UP_WEAK_RIGHT:
            Add_Line(0,1,strong_angle_left_up.row-5,(uint8)line_left[strong_angle_left_up.row-5],strong_angle_left_up.row,strong_angle_left_up.col);
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,weak_angle_right.row,weak_angle_right.col);
            break;
        case STRONG_UP_TWO:
            Add_Line(0,1,strong_angle_left_up.row-5,(uint8)line_left[strong_angle_left_up.row-5],strong_angle_left_up.row,strong_angle_left_up.col);
            Add_Line(1,1,strong_angle_right_up.row-5,(uint8)line_right[strong_angle_right_up.row-5],strong_angle_right_up.row,strong_angle_right_up.col);
            break;
        case STRONG_DOWN_TWO:
            Add_Line(0,2,strong_angle_left_down.row,strong_angle_left_down.col,strong_angle_left_down.row+5,(uint8)line_left[strong_angle_left_down.row+5]);
            Add_Line(1,2,strong_angle_right_down.row,strong_angle_right_down.col,strong_angle_right_down.row+5,(uint8)line_right[strong_angle_right_down.row+5]);
            break;
        case STRONG_LEFT_UP_STRONG_LEFT_DOWN:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,strong_angle_left_down.row,strong_angle_left_down.col);
            break;
        case STRONG_RIGHT_UP_STRONG_RIGHT_DOWN:
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,strong_angle_right_down.row,strong_angle_right_down.col);
            break;
        case STRONG_LEFT_UP_WEAK_LEFT:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,weak_angle_left.row,weak_angle_left.col);
            break;
        case STRONG_RIGHT_UP_WEAK_RIGHT:
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,weak_angle_right.row,weak_angle_right.col);
            break;
        case STRONG_LEFT_WEAK_RIGHT:
            Add_Line(0,0,strong_angle_left_up.row,strong_angle_left_up.col,strong_angle_left_down.row,strong_angle_left_down.col);
            Add_Line(1,2,weak_angle_right.row,weak_angle_right.col,weak_angle_right.row+5,(uint8)line_right[weak_angle_right.row+5]);
            break;
        case STRONG_RIGHT_WEAK_LEFT:
            Add_Line(0,2,weak_angle_left.row,weak_angle_left.col,weak_angle_left.row+5,(uint8)line_left[weak_angle_left.row+5]);
            Add_Line(1,0,strong_angle_right_up.row,strong_angle_right_up.col,strong_angle_right_down.row,strong_angle_right_down.col);
            break;
        default:
            break;
    }
}

Curve_enum Curve_Judge(void)
{
    return NO_CURVE;
}

Round_enum Round_Judge(void)
{
    uint8 l_s,r_s;
    l_s=Calculate_Linear_Degree(line_left);
    r_s=Calculate_Linear_Degree(line_right);
    if(l_s==1&&r_s==0&&Find_Strong_Angle_Right_Down(15,MT9V03X_H-20)==1)
    {
        round_state=ROUND_READY;
        round_size=round_size_full%10;
        round_size_full/=10;
        return ROUND_R;
    }
    else if(r_s==1&&l_s==0&&Find_Strong_Angle_Left_Down(15,MT9V03X_H-20)==1)
    {
        round_state=ROUND_READY;
        round_size=round_size_full%10;
        round_size_full/=10;
        return ROUND_L;
    }
    else
        return NO_ROUND;
}

void Round_State_Update(void)
{
    switch(round_state)
    {
        case ROUND_READY:
            if(round_position==ROUND_L)
            {
                if(!Find_Strong_Angle_Left_Down(15,MT9V03X_H-20))
                    round_state=GET_IN_READY;
            }
            else if(round_position==ROUND_R)
            {
                if(!Find_Strong_Angle_Right_Down(15,MT9V03X_H-20))
                    round_state=GET_IN_READY;
            }
            break;
        case GET_IN_READY:
            if(round_position==ROUND_L)
            {
                if(Find_Strong_Angle_Left_Up(10,20))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=GET_IN_ROUND;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
            {
                if(Find_Strong_Angle_Right_Up(10,20))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=GET_IN_ROUND;
                    round_state_update_count=0;
                }
            }
            break;
        case GET_IN_ROUND:
            if(round_position==ROUND_L)
            {
                if(!Find_Strong_Angle_Left_Up(12,MT9V03X_H-10))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=IN_ROUND;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
            {
                if(!Find_Strong_Angle_Right_Up(12,MT9V03X_H-10))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=IN_ROUND;
                    round_state_update_count=0;
                }
            }
            break;
        case IN_ROUND:
            if(round_position==ROUND_L)
            {
                if(Find_Strong_Angle_Right_Down(10,MT9V03X_H-15) || Find_Weak_Angle_Right(10,MT9V03X_H-15))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=OUTTING;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
            {
                if(Find_Strong_Angle_Left_Down(10,MT9V03X_H-15) || Find_Weak_Angle_Left(10,MT9V03X_H-15))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=OUTTING;
                    round_state_update_count=0;
                }
            }
            break;
        case OUTTING:
            if(round_position==ROUND_L)
            {
                if(Find_Strong_Angle_Right_Down(10,MT9V03X_H-10)==0 && Find_Weak_Angle_Right(10,MT9V03X_H-10)==0)
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=GET_OUT_ROUND;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
            {
                if(Find_Strong_Angle_Left_Down(10,MT9V03X_H-10)==0 && Find_Weak_Angle_Left(10,MT9V03X_H-10)==0)
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=GET_OUT_ROUND;
                    round_state_update_count=0;
                }
            }
            break;
        case GET_OUT_ROUND:
            if(round_position==ROUND_L)
            {
                if(Find_Strong_Angle_Left_Up(10,MT9V03X_H-10))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=OUT_ROUND;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
            {
                if(Find_Strong_Angle_Right_Up(10,MT9V03X_H-10))
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {
                    round_state=OUT_ROUND;
                    round_state_update_count=0;
                }
            }
            break;
        case OUT_ROUND:
            if(round_position==ROUND_L)
            {
                if(Find_Strong_Angle_Left_Up(10,MT9V03X_H-10)==0)
                    round_state_update_count++;
                else
                    round_state_update_count=0;
                if(round_state_update_count>=3)
                {

                    round_position=NO_ROUND;
                    round_state=ROUND_READY;
                    element_state=STRAIGHT;
//                    dir.p+=round_p_offset;
                    round_state_update_count=0;
                }
            }
            else if(round_position==ROUND_R)
                round_state_update_count++;
            else
                round_state_update_count=0;
            if(round_state_update_count>=3)
            {
                if(Find_Strong_Angle_Right_Up(10,MT9V03X_H-10)==0){
                    round_position=NO_ROUND;
                    round_state=ROUND_READY;
                    element_state=STRAIGHT;
//                    dir.p+=round_p_offset;
                    round_state_update_count=0;
                }
            }
            break;
    }
}

void Round_Addline(void)
{
    switch(round_state)
    {
        case ROUND_READY:
            if(round_position==ROUND_L)
                Add_Line(0,2,
                        strong_angle_left_down.row,
                        strong_angle_left_down.col,
                        strong_angle_left_down.row+8,
                        (uint8)line_left[strong_angle_left_down.row+8]);
            else if(round_position==ROUND_R)
                Add_Line(1,2,
                        strong_angle_right_down.row,
                        strong_angle_right_down.col,
                        strong_angle_right_down.row+8,
                        (uint8)line_right[strong_angle_right_down.row+8]);
            break;
        case GET_IN_READY:
            if(round_position==ROUND_L)
            {
                if(Find_Left_Round_Point(5,40)==1)
                {
                    Add_Line_Left_By_Round_Point(10,(uint8)line_right[10],MT9V03X_H-10,(uint8)line_right[MT9V03X_H-10]);
                }
            }
            else if(round_position==ROUND_R)
            {
                if(Find_Right_Round_Point(5,40)==1)
                {
                    Add_Line_Right_By_Round_Point(10,(uint8)line_left[10],MT9V03X_H-10,(uint8)line_left[MT9V03X_H-10]);
                }
            }
            break;
        case GET_IN_ROUND:
            if(round_state_update_count==0)
            {
                if(round_position==ROUND_L)
                {
                   Add_Line_Left_Round_Up_Point(strong_angle_left_up);
                }
                else if(round_position==ROUND_R)
                {
                   Add_Line_Right_Round_Up_Point(strong_angle_right_up);
                }
            }
            break;
        case IN_ROUND:
            break;
        case OUTTING:
            if(round_position==ROUND_L)
            {
                Boundary_Left_Init();
                if(Find_Strong_Angle_Right_Down(5,MT9V03X_H-5))
                {
                    Add_Line(1,0,
                            0,
                            40,
                            strong_angle_right_down.row,
                            strong_angle_right_down.col);
                    round_out_point_row=strong_angle_right_down.row;
                    round_out_point_col=strong_angle_right_down.col;
                }
                else if(Find_Weak_Angle_Right(5,MT9V03X_H-5))
                {
                    Add_Line(1,0,
                            0,
                            40,
                            weak_angle_right.row,
                            weak_angle_right.col);
                    round_out_point_row=weak_angle_right.row;
                    round_out_point_col=weak_angle_right.col;
                }
            }
            else if(round_position==ROUND_R)
            {
                Boundary_Right_Init();
                if(Find_Strong_Angle_Left_Down(5,MT9V03X_H-5))
                {
                    Add_Line(0,0,
                            0,
                            MT9V03X_W-40,
                            strong_angle_left_down.row,
                            strong_angle_left_down.col);
                    round_out_point_row=strong_angle_left_down.row;
                    round_out_point_col=strong_angle_left_down.col;
                }
                else if(Find_Weak_Angle_Left(5,MT9V03X_H-5))
                {
                    Add_Line(0,0,
                            0,
                            MT9V03X_W-40,
                            weak_angle_left.row,
                            weak_angle_left.col);
                    round_out_point_row=weak_angle_left.row;
                    round_out_point_col=weak_angle_left.col;
                }
            }
            break;
        case GET_OUT_ROUND:
            if(round_position==ROUND_L)
            {
                Boundary_Left_Init();
//                if(line_right[MT9V03X_H-15]==LINE_INVALID_RIGHT||line_right[15]==LINE_INVALID_RIGHT
//                        ||Calculate_Slope(15,(uint8)line_right[15],MT9V03X_H-15,(uint8)line_right[MT9V03X_H-15])>2.5)
////                Add_Line(1,0,0,25,MT9V03X_H-10,MT9V03X_W-1);
//                    Add_Line(1,0,0,50,round_out_point_row,round_out_point_col);
            }
            else if(round_position==ROUND_R)
            {
                Boundary_Right_Init();
//                if(line_left[MT9V03X_H-15]==LINE_INVALID_LEFT||line_left[15]==LINE_INVALID_LEFT
//                        ||Calculate_Slope(15,(uint8)line_left[15],MT9V03X_H-15,(uint8)line_left[MT9V03X_H-15])<-2.5)
////                Add_Line(0,0,0,MT9V03X_W-25,MT9V03X_H-10,0);
//                    Add_Line(0,0,0,MT9V03X_W-50,round_out_point_row,round_out_point_col);
            }
            break;
        case OUT_ROUND:
            if(round_position==ROUND_L)
            {
                Add_Line(0,1,
                        strong_angle_left_up.row-8,
                        (uint8)line_left[strong_angle_left_up.row-8],
                        strong_angle_left_up.row,
                        strong_angle_left_up.col);
            }
            else if(round_position==ROUND_R)
            {
                Add_Line(1,1,
                        strong_angle_right_up.row-8,
                        (uint8)line_right[strong_angle_right_up.row-8],
                        strong_angle_right_up.row,
                        strong_angle_right_up.col);
            }
            break;
    }
}

uint8 Zebra_Judge(void){
    uint8 i,j,count=0;
    uint8 color=0;
    uint8 zebra_judge=0;//斑马线判断
    for(i=20;i<MT9V03X_H-1;i++){
        for(j=40;j<140;j++){
            if(binary_img_data[i][j]==BLACK_IMG&&color==0){
                color=1;
                count++;
            }
            else if(binary_img_data[i][j]==WHITE_IMG&&color==1){
                color=0;
                count++;
            }
            else
                continue;
        }
        if(count>=8){
            count=0;
            zebra_judge=1;
            zebra_row=i;
            return 1;
        }
        else{
            count=0;
        }
    }
    return 0;
}

void Zebra_Update(void)
{
    switch(zebra_state)
    {
        case ZEBRA_START:
            if(!Zebra_Judge())
            {
                zebra_state_update_count++;
            }
            else zebra_state_update_count=0;
            if(zebra_state_update_count>=5)
            {
                zebra_state=ZEBRA_FIND;
                element_state=STRAIGHT;
            }
            break;
        case ZEBRA_FIND:
            if(!Zebra_Judge())
            {
                zebra_state_update_count++;
            }
            else zebra_state_update_count=0;
            if(zebra_state_update_count>=2)
            {
                zebra_state=ZEBRA_MAINTAIN;
                zebra_time_flag=1;
            }
            break;
        case ZEBRA_MAINTAIN:
            if(zebra_time_flag==0)
            {
                zebra_state=ZEBRA_STOP;
                LIGHT(0);
            }
            zebra_time_flag=1;
            break;
        case ZEBRA_STOP:
            if(zebra_time_flag==0)
            {
                zebra_state=ZEBRA_START;
                element_state=IDLE;
            }
            break;
    }
}

void Element_Judge(void)
{
    Outside_Bound_Judge();
    switch(element_state)
    {
        case IDLE:
            break;
        case STRAIGHT:
            if(Zebra_Judge()){
                zebra_state_update_count++;
            }
            else if((round_position=Round_Judge())!=NO_ROUND)
            {
                element_state=ROUND;
//                dir.p-=round_p_offset;
            }
            else if((crossing_position=Crossing_Judge())!=NO_CROSS && !Zebra_Judge())
                element_state=CROSSING;
            else
                zebra_state_update_count=0;
            if(zebra_state_update_count>=3)
            {
                element_state=ZEBRA;
                zebra_state_update_count=0;
            }
            break;
        case CURVE:
            break;
        case CROSSING:
            if((crossing_position=Crossing_Judge())==NO_CROSS)
                element_state=STRAIGHT;
            break;
        case ROUND:
            Round_State_Update();
            break;
        case ZEBRA:
            Zebra_Update();
            break;
    }
}

void Element_Deal(void)
{
    switch(element_state)
    {
        case IDLE:
            if(start_flag)
            {
                LIGHT(0);
                start_flag=0;
            }
            BUZZER(0);
            break;
        case STRAIGHT:
            BUZZER(0);
            break;
        case CURVE:
            BUZZER(0);
            break;
        case CROSSING:
            Crossing_Add_Line();
            BUZZER(1);
            break;
        case ROUND:
            Round_Addline();
            BUZZER(1);
            break;
        case ZEBRA:
            if(zebra_state==ZEBRA_START || zebra_state==ZEBRA_FIND)
            Add_Line_Zebra(5,MT9V03X_H-5);
            BUZZER(1);
            break;
    }
}







