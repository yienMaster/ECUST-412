#include "image.h"
#include "zf_common_headfile.h"

//图像初始的变量*******************
uint8 img_data[MT9V03X_H][MT9V03X_W];   //存放原始图像
Point_Structure point_up,point_down,point_middle;   //上下亮点和中点
uint8 point_distance,target_point_distance;   //上下两点间距

//判断亮点的偏移值
int16 point_detect_offset_row[8]={0,-1,-1,-1,0,1,1,1};
int16 point_detect_offset_col[8]={-1,-1,0,1,1,1,0,-1};

uint8 stop_count,stop_flag;     //丢点时间计数和标志位
uint8 find_point_flag;          //找点标志位




uint8 Find_Point(void)
{
    int16 i,j,k;
    uint8 find_up_up_flag,find_up_down_flag,find_down_up_flag,find_down_down_flag,bright_num;
    int16 row_up_up,row_up_down,col_up_up,col_up_down,row_down_up,row_down_down,col_down_up,col_down_down;
    int16 point_up_row,point_up_col,point_down_row,point_down_col;
    for(i=1;i<MT9V03X_H-1;i++)
    {
        for(j=1;j<MT9V03X_W-1;j++)
        {
            find_up_up_flag=find_up_down_flag=find_down_up_flag=find_down_down_flag=0;
            bright_num=0;
            if(img_data[i][j]>=BRIGHTNESS_MIDDLE_THRESHOLD)
            {
                for(k=0;k<8;k++)
                    if(img_data[i+point_detect_offset_row[k]][j+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
                        bright_num++;
                if(bright_num>=3)
                {
                    row_up_up=i;
                    col_up_up=j;
                    find_up_up_flag=1;
                }
            }
            if(find_up_up_flag)
            {
                for(row_up_down=row_up_up+4;row_up_down>=row_up_up;row_up_down--)
                {
                    for(col_up_down=col_up_up+6;col_up_down>=col_up_up;col_up_down--)
                    {
                        bright_num=0;
                        if(img_data[row_up_down][col_up_down]>=BRIGHTNESS_MIDDLE_THRESHOLD)
                        {
                            for(k=0;k<8;k++)
                                if(img_data[row_up_down+point_detect_offset_row[k]][col_up_down+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
                                    bright_num++;
                            if(bright_num>=3)
                            {
                                point_up_row=(row_up_up+row_up_down)>>1;
                                point_up_col=(col_up_up+col_up_down)>>1;
                                find_up_down_flag=1;
                                break;
                            }
                        }
                    }
                    if(find_up_down_flag) break;
                }
            }
            if(find_up_down_flag)
            {
                for(row_down_up=row_up_down+6;row_down_up<MT9V03X_W-10;row_down_up++)
                {
                    for(col_down_up=point_up_col-15;col_down_up<=point_up_col+15;col_down_up++)
                    {
                        bright_num=0;
                        if(img_data[row_down_up][col_down_up]>=BRIGHTNESS_MIDDLE_THRESHOLD)
                        {
                            for(k=0;k<8;k++)
                                if(img_data[row_down_up+point_detect_offset_row[k]][col_down_up+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
                                    bright_num++;
                            if(bright_num>=3)
                            {
                                find_down_up_flag=1;
                                break;
                            }
                        }
                    }
                    if(find_down_up_flag) break;
                }
            }
            if(find_down_up_flag)
            {
                for(row_down_down=row_down_up+4;row_down_down>=row_down_up;row_down_down--)
                {
                    for(col_down_down=col_down_up+6;col_down_down>=col_down_up;col_down_down--)
                    {
                        bright_num=0;
                        if(img_data[row_down_down][col_down_down]>=BRIGHTNESS_MIDDLE_THRESHOLD)
                        {
                            for(k=0;k<8;k++)
                                if(img_data[row_down_down+point_detect_offset_row[k]][col_down_down+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
                                    bright_num++;
                            if(bright_num>=3)
                            {
                                point_down_row=(row_down_up+row_down_down)>>1;
                                point_down_col=(col_down_up+col_down_down)>>1;
                                find_down_down_flag=1;
                                break;
                            }
                        }
                    }
                    if(find_down_down_flag) break;
                }
            }
            if(find_down_down_flag) break;
        }
        if(find_down_down_flag) break;
    }
    if(find_down_down_flag)
    {
        point_up.row=point_up_row; point_up.col=point_up_col;
        point_down.row=point_down_row; point_down.col=point_down_col;
        point_middle.row=(point_up.row+point_down.row)>>1;
        point_middle.col=(point_up.col+point_down.col)>>1;
        point_distance=point_down.row-point_up.row;
        stop_count=0;
        stop_flag=0;
        return 1;
    }
    else
    {
        if(run_state==2)
            stop_flag=1;
        return 0;
    }
}



//void Find_Point(void)
//{
//    int16 i,j,k;
//    int16 row_up,col_up,row_down,col_down;
//    uint8 find_up_flag,bright_num;
//    int16 point_up_row,point_up_col,point_down_row,point_down_col;
//    uint8 state;
//    //中间一列开始向两侧遍历
//    for(j=0;j<(MT9V03X_W>>1)-1;j++)
//    {
//        //右边列
//        find_up_flag=0; find_down_flag=0; col_up=(MT9V03X_W>>1)+j; state=0;
//        for(i=1;i<MT9V03X_H-1;i++)
//        {
//            bright_num=0;
//            switch(state)
//            {
//                case 0:
//                    row_up=i;
//                    if(img_data[row_up][col_up]>=BRIGHTNESS_MIDDLE_THRESHOLD)
//                    {
//                        for(k=0;k<8;k++)
//                            if(img_data[row_up+point_detect_offset_row[k]][col_up+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
//                                bright_num++;
//                        if(bright_num>=3)
//                        {
//                            state=1;
//                            find_up_flag=1;
//                            point_up_row=row_up;
//                            point_up_col=col_up;
//                        }
//                    }
//                    break;
//                case 1:
//                    row_down=i;
//                    for(col_down=Max(point_up_col-14,1);col_down<Min(point_up_col+15,MT9V03X_W-1);col_down++)
//                    {
//                        if(img_data[row_down][col_down]>=BRIGHTNESS_MIDDLE_THRESHOLD)
//                        {
//                            for(k=0;k<8;k++)
//                                if(img_data[row_down+point_detect_offset_row[k]][col_down+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
//                                    bright_num++;
//                            if(bright_num>=3 && row_down-point_up_row>10)
//                            {
//                                find_down_flag=1;
//                                point_down_row=row_down;
//                                point_down_col=col_down;
//                            }
//                        }
//                    }
//                    break;
//            }
//            if(find_down_flag) break;
//        }
//        if(find_down_flag) break;
//        //左边列
//        find_up_flag=0; find_down_flag=0; col_up=(MT9V03X_W>>1)-j; state=0;
//        for(i=1;i<MT9V03X_H-1;i++)
//        {
//            bright_num=0;
//            switch(state)
//            {
//                case 0:
//                    row_up=i;
//                    if(img_data[row_up][col_up]>=BRIGHTNESS_MIDDLE_THRESHOLD)
//                    {
//                        for(k=0;k<8;k++)
//                            if(img_data[row_up+point_detect_offset_row[k]][col_up+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
//                                bright_num++;
//                        if(bright_num>=3)
//                        {
//                            state=1;
//                            find_up_flag=1;
//                            point_up_row=row_up;
//                            point_up_col=col_up;
//                        }
//                    }
//                    break;
//                case 1:
//                    row_down=i;
//                    for(col_down=Max(point_up_col-14,1);col_down<Min(point_up_col+15,MT9V03X_W-1);col_down++)
//                    {
//                        if(img_data[row_down][col_down]>=BRIGHTNESS_MIDDLE_THRESHOLD)
//                        {
//                            for(k=0;k<8;k++)
//                                if(img_data[row_down+point_detect_offset_row[k]][col_down+point_detect_offset_col[k]]>=BRIGHTNESS_SURROUNDING_THRESHOLD)
//                                    bright_num++;
//                            if(bright_num>=3 && row_down-point_up_row>10)
//                            {
//                                find_down_flag=1;
//                                point_down_row=row_down;
//                                point_down_col=col_down;
//                            }
//                        }
//                    }
//                    break;
//            }
//            if(find_down_flag) break;
//        }
//        if(find_down_flag) break;
//    }
//    if(find_down_flag)
//        {
//            point_up.row=point_up_row; point_up.col=point_up_col;
//            point_down.row=point_down_row; point_down.col=point_down_col;
//            point_middle.row=(point_up.row+point_down.row)>>1;
//            point_middle.col=(point_up.col+point_down.col)>>1;
//            count=0;
//            time_flag=0;
//        }
//        else
//        {
//            time_flag=1;
//        }
//    point_distance=point_down.row-point_up.row;
//}



/*
#define minDistance  (5)

void findTwoBrightestPoint(void)
{
    int maxBrightness = -1;  // 初始最大亮度为负值
    int secondMaxBrightness = -1; // 次大亮度为负值
    int brightestRow = -1, brightestCol = -1;
    int secondBrightestRow = -1, secondBrightestCol = -1;

    // 遍历图像的每个像素
    for (int row = 0; row < MT9V03X_H; row++)
    {
        for (int col = 0; col < MT9V03X_W; col++)
        {
            int brightness = img_data[row][col];

            // 更新最大亮度和亮点坐标
            if (brightness > maxBrightness)
            {
                // 更新第二大亮度为原来的最大亮度
              if (abs(brightestRow - row) > minDistance || abs(brightestCol - col) > minDistance)
              {
                secondMaxBrightness = maxBrightness;
                secondBrightestRow = brightestRow;
                secondBrightestCol = brightestCol;
              }

                // 更新最大亮度
                maxBrightness = brightness;
                brightestRow = row;
                brightestCol = col;
            }
        }
    }
    // 输出结果
    if (brightestRow != -1 && brightestCol != -1)
    {
       point_up.row=brightestRow,point_up.col=brightestCol;
    }
    if (secondBrightestRow != -1 && secondBrightestCol != -1)
    {
       point_down.row=secondBrightestRow, point_down.col=secondBrightestCol;
    }
}
*/



