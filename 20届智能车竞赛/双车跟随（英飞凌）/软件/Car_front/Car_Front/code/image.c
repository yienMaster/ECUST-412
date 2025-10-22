#include "image.h"
#include "zf_common_headfile.h"

//ͼ���ʼ�ı���*******************
uint8 img_data[MT9V03X_H][MT9V03X_W];//���ԭʼͼ��
uint8 binary_img_data[MT9V03X_H][MT9V03X_W];//��Ŷ�ֵ��ͼ��
uint8 ostu_threshold;//��ֵ����ֵ


//���߱���*******************
int16 line_left[MT9V03X_H],line_right[MT9V03X_H],line_middle[MT9V03X_H];

//�����������еı���*******************
Longest_Col_Structure longest_col_left,longest_col_right;
uint8 white_col_mid_row;



//ͼ�������������ֵ*******************
uint8 Otsu_Threshold(uint8 *image, uint16 col, uint16 row)
{
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
    double OmegaBack=0, OmegaFore=0, MicroBack=0, MicroFore=0, SigmaB=0, Sigma=0; // ��䷽��;
    uint8 MinValue=0, MaxValue=0;
    uint8 threshold = 0;
    for (Y = 0; Y <Image_Height; Y++) //Y<Image_Height��ΪY =Image_Height���Ա���� �ж�ֵ��
    {
        //Y=Image_Height;
        for (X = 0; X < Image_Width; X++)
        {
        HistGram[(int)data[Y*Image_Width + X]]++; //ͳ��ÿ���Ҷ�ֵ�ĸ�����Ϣ
        }
    }
    for (MinValue = 0; MinValue < 256 && HistGram[MinValue] == 0; MinValue++) ;        //��ȡ��С�Ҷȵ�ֵ
    for (MaxValue = 255; MaxValue > MinValue && HistGram[MinValue] == 0; MaxValue--) ; //��ȡ���Ҷȵ�ֵ

    if (MaxValue == MinValue)
    {
        return MaxValue;          // ͼ����ֻ��һ����ɫ
    }
    if (MinValue + 1 == MaxValue)
    {
        return MinValue;      // ͼ����ֻ�ж�����ɫ
    }

    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        Amount += HistGram[Y];        //  ��������
    }

    PixelIntegral = 0;
    for (Y = MinValue; Y <= MaxValue; Y++)
    {
        PixelIntegral += HistGram[Y] * Y;//�Ҷ�ֵ����
    }
    SigmaB = -1;
    for (Y = MinValue; Y < MaxValue; Y++)
    {
          PixelBack = PixelBack + HistGram[Y];    //ǰ�����ص���
          PixelFore = Amount - PixelBack;         //�������ص���
          OmegaBack = (double)PixelBack / Amount;//ǰ�����ذٷֱ�
          OmegaFore = (double)PixelFore / Amount;//�������ذٷֱ�
          PixelIntegralBack += HistGram[Y] * Y;  //ǰ���Ҷ�ֵ
          PixelIntegralFore = PixelIntegral - PixelIntegralBack;//�����Ҷ�ֵ
          MicroBack = (double)PixelIntegralBack / PixelBack;//ǰ���ҶȰٷֱ�
          MicroFore = (double)PixelIntegralFore / PixelFore;//�����ҶȰٷֱ�
          Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);//g
          if (Sigma > SigmaB)//����������䷽��g
          {
              SigmaB = Sigma;
              threshold = (uint8)Y;
          }
    }
   return threshold;
}


//����ͼ�񣬽��ж�ֵ��*******************
void IMG_Init(void)
{
    uint8 i,j;

    memcpy(img_data[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);//����ͼ��
    ostu_threshold=Otsu_Threshold(img_data[0], MT9V03X_W, MT9V03X_H);
    //ostu_threshold=120;
    for(i=0;i<MT9V03X_H;i++)
    {
        for(j=0;j<MT9V03X_W;j++)
        {
            if(img_data[i][j] >= ostu_threshold)
            {
                binary_img_data[i][j]=WHITE_IMG;//��ֵ��
            }
            else
            {
                binary_img_data[i][j]=BLACK_IMG;
            }
        }
    }
}



//��ʼ���߽���*******************
void Line_Init(void)
{
    uint8 i;
    for(i=0;i<MT9V03X_H;i++)
    {
        line_left[i]=LINE_INVALID_LEFT;
        line_right[i]=LINE_INVALID_RIGHT;
        line_middle[i]=LINE_INVALID_MIDDLE;
    }
    longest_col_left.col=0;
    longest_col_left.highest_row=MT9V03X_H-1;
    longest_col_right.col=MT9V03X_W-1;
    longest_col_right.highest_row=MT9V03X_H-1;
    strong_angle_left_up.row=0; strong_angle_left_up.col=0;
    strong_angle_left_down.row=0; strong_angle_left_down.col=0;
    strong_angle_right_up.row=0; strong_angle_right_up.col=0;
    strong_angle_right_down.row=0; strong_angle_right_down.col=0;
    weak_angle_left.row=0; weak_angle_left.col=0;
    weak_angle_right.row=0; weak_angle_right.col=0;
    white_col_mid_row=MT9V03X_H-1;
}


//�������*******************
void Longest_Col(void)
{
    uint8 i,j;
    uint8 white_count,row_start,white_count_max=0;
    for(i=6;i<MT9V03X_W-6;i++)
    {
        white_count=0; row_start=MT9V03X_H-1;
        for(j=MT9V03X_H-1;j>0;j--)
        {
            if(binary_img_data[j][i]==WHITE_IMG)
            {
                white_count++;
                if(white_count==1)  //��һ���׿��Ӧ��Ϊ��ʼ��
                    row_start=j;
            }
            if(white_count!=0 && binary_img_data[j][i]==BLACK_IMG)  //�ҵ��ڿ�
                break;
            if(j<MT9V03X_H-10 && white_count==0)    //�·������ڿ�
                break;
        }
        if(i==MT9V03X_W>>1)
            white_col_mid_row=row_start-white_count+1;
        if(white_count>white_count_max) //���������
        {
            white_count_max=white_count;
            longest_col_left.col=i;
            longest_col_left.highest_row=row_start-white_count+1;
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
        else if(white_count==white_count_max)   //��������
        {
            longest_col_right.col=i;
            longest_col_right.highest_row=row_start-white_count+1;
        }
    }
//    tft180_show_uint(100,0,longest_col_left.col,3);
//    tft180_show_uint(100,10,longest_col_right.col,3);
//    tft180_show_uint(100,20,longest_col_left.highest_row,3);
//    tft180_show_uint(100,30,longest_col_right.highest_row,3);
//    tft180_show_uint(100,40,Min(longest_col_left.highest_row,longest_col_right.highest_row),3);
//    tft180_show_uint(100,50,white_count_max,3);
}



//Ѱ�������************************
void Left_Line(void)
{
    uint8 i,j;
    for(i=MT9V03X_H-1;i>Min(longest_col_left.highest_row,longest_col_right.highest_row);i--)
    {
        for(j = (uint8)Constrain_int16(longest_col_left.col+5,MT9V03X_W-1,0);j >= 3;j--)
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

void Right_Line(void)
{
    uint8 i,j;
    for(i=MT9V03X_H-1;i>Min(longest_col_left.highest_row,longest_col_right.highest_row);i--)
    {
        for(j = (uint8)Constrain_int16(longest_col_right.col-5,MT9V03X_W-1,0);j <=MT9V03X_W-4;j++)
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


uint8 Find_Middle_Line_Weak_Angle_Right(uint8 start_row)
{
    uint8 i=0,angle_row,angle_col;
    for(i=start_row;i<MT9V03X_H-10;i++)
    {
        if(line_middle[i-3]!=LINE_INVALID_MIDDLE
         &&line_middle[i-2]!=LINE_INVALID_MIDDLE
         &&line_middle[i-1]!=LINE_INVALID_MIDDLE
         &&line_middle[i]!=LINE_INVALID_MIDDLE
         &&line_middle[i+1]!=LINE_INVALID_MIDDLE
         &&line_middle[i+2]!=LINE_INVALID_MIDDLE
         &&line_middle[i+3]!=LINE_INVALID_MIDDLE
         &&line_middle[i]>4 && line_middle[i]<MT9V03X_W-5
         &&(line_middle[i-3]-line_middle[i-2]<0)
         &&(line_middle[i-2]-line_middle[i-1]<=0)
         &&(line_middle[i-1]-line_middle[i]<=0)
         &&(line_middle[i]-line_middle[i+1]>=0)
         &&(line_middle[i+1]-line_middle[i+2]>=0)
         &&(line_middle[i+2]-line_middle[i+3]>0))
        {
            angle_row=i;
            angle_col=(uint8)line_middle[i];
            Record_point(angle_row,angle_col,1);
            return 1;
        }
    }
    return 0;
}

uint8 Find_Middle_Line_Weak_Angle_Left(uint8 start_row)
{
    uint8 i=0,angle_row,angle_col;
    for(i=start_row;i<MT9V03X_H-10;i++)
    {
        if(line_middle[i-3]!=LINE_INVALID_MIDDLE
         &&line_middle[i-2]!=LINE_INVALID_MIDDLE
         &&line_middle[i-1]!=LINE_INVALID_MIDDLE
         &&line_middle[i]!=LINE_INVALID_MIDDLE
         &&line_middle[i]>4 && line_middle[i]<MT9V03X_W-5
         &&line_middle[i+1]!=LINE_INVALID_MIDDLE
         &&line_middle[i+2]!=LINE_INVALID_MIDDLE
         &&line_middle[i+3]!=LINE_INVALID_MIDDLE
         &&(line_middle[i-3]-line_middle[i-2]>0)
         &&(line_middle[i-2]-line_middle[i-1]>=0)
         &&(line_middle[i-1]-line_middle[i]>=0)
         &&(line_middle[i]-line_middle[i+1]<=0)
         &&(line_middle[i+1]-line_middle[i+2]<=0)
         &&(line_middle[i+2]-line_middle[i+3]<0))
        {
            angle_row=i;
            angle_col=(uint8)line_middle[i];
            Record_point(angle_row,angle_col,1);
            return 1;
        }
    }
    return 0;
}

uint8 Add_Middle_Line_Mode_Left(uint8 row)
{
    uint8 i,row_left=MT9V03X_H-10;
    for(i=MT9V03X_H-10;i>row;i--)
    {
        if(line_middle[i]<line_middle[row_left])
        {
            row_left=i;
        }
        if(line_middle[i]-line_middle[row_left]>6 && line_middle[MT9V03X_H-1]-line_middle[row_left]>4)
            return row_left;
    }
    return 0;
}

uint8 Add_Middle_Line_Mode_Right(uint8 row)
{
    uint8 i,row_right=MT9V03X_H-10;
    for(i=MT9V03X_H-10;i>row;i--)
    {
        if(line_middle[i]>line_middle[row_right])
        {
            row_right=i;
        }
        if(line_middle[i]-line_middle[row_right]<-6 && line_middle[MT9V03X_H-1]-line_middle[row_right]<-4)
            return row_right;
    }
    return 0;
}


//������*******************
void Middle_Line(void)
{
    uint8 i,row=0,row2=0;
    float k,b;
    for(i=MT9V03X_H-1;i>0;i--)
    {
        if(line_left[i]==LINE_INVALID_LEFT && line_right[i]==LINE_INVALID_RIGHT)  line_middle[i]=LINE_INVALID_MIDDLE;
        else if(line_left[i]==LINE_INVALID_LEFT) line_middle[i]=line_right[i]>>1;
        else if(line_right[i]==LINE_INVALID_RIGHT) line_middle[i]=(line_left[i]+MT9V03X_W)>>1;
        else line_middle[i]=(line_left[i]+line_right[i])>>1;
    }
    for(i=MT9V03X_H-40;i>15;i--)
    {
        if(Abs(line_middle[i]-line_middle[i-1])>5 && line_middle[i-1]!=LINE_INVALID_MIDDLE)
        {
            row=i;
            break;
        }
    }
    if(row!=0 && element_state==STRAIGHT)
    {
        if((row2=Add_Middle_Line_Mode_Left(row))!=0)
        {
            k=Calculate_Slope(row2,(uint8)line_middle[row2],row2+5,(uint8)line_middle[row2+5]);
            b=line_middle[row2]-k*row2;
            for(i=row2-1;i>0;i--)
            {
                line_middle[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }
        else if((row2=Add_Middle_Line_Mode_Right(row))!=0)
        {
            k=Calculate_Slope(row2,(uint8)line_middle[row2],row2+5,(uint8)line_middle[row2+5]);
            b=line_middle[row2]-k*row2;
            for(i=row2-1;i>0;i--)
            {
                line_middle[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
            }
        }

//        else
//            k=Calculate_Slope(row,(uint8)line_middle[row],row+5,(uint8)line_middle[row+5]);
//        b=line_middle[row]-k*row;
//        for(i=row-1;i>0;i--)
//        {
//            line_middle[i]=Constrain_int16((int16)(k*i+b),MT9V03X_W-1,0);
////            line_middle[i]=line_middle[MT9V03X_H-1];
//        }
    }

}


void Image_Deal(void)
{
    IMG_Init();
    Line_Init();
    Longest_Col();
    Left_Line();
    Right_Line();
//    Calculate_Linear_Degree(line_left);
//    Calculate_Linear_Degree(line_right);
    Element_Judge();
    Element_Deal();
    Middle_Line();
}



