//-------------------------------------------------------------------------------------------------------------------
//  ���:������ͼ����

//------------------------------------------------------------------------------------------------------------------
#include "image.h"
#include "zf_common_headfile.h"
int eresting=1;
float border = 94;
int buzzer = 0;
int cross_sum = 0;
int sum_island = 0;
//island 1Ϊ�󻷵���2Ϊ�һ���
int island=0;
/*******�����ű�������***********/
SingleBridgeState  BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;
int jump_position = 0;
int stop_position = 0;
int monotonicity_line_l =0;
int monotonicity_line_r =0;
int string_not = 0;
int jump_h = 40;
int m=0;
int n=0;
int s=0;
int c=0;
int v=0;
int e=0;
int f=0;
int q=0;
int j=0;
int u=0;
/*
�������ƣ�int my_limit(int num,int value)
����˵���������޷�
����˵����
�������أ�a <= c <= b
�޸�ʱ�䣺2025��3��14��
��    ע��
example��  my_auu(c,b,a)��
 */
int my_auu(int c,int b,int a)
 {
     if(c > b)
             return b;
     else if(c < a)
             return a;
     else
         return c;
 }
/*
�������ƣ�int my_limit(int num,int value)
����˵�����޷�
����˵����
�������أ��޷�ֵ
�޸�ʱ�䣺2025��3��14��
��    ע��
example��  my_abslimit( num,value)��
 */
int my_limit(int num,int value)
{
    if(value < num)
        return 0;
    else
        return value;
}
/*
�������ƣ�int my_abs(int value)
����˵���������ֵ
����˵����
�������أ�����ֵ
�޸�ʱ�䣺2025��1��5��
��    ע��
example��  my_abs( x)��
 */
int my_abs(int value)
{
if(value>=0) return value;
else return -value;
}

int limit_a_b(int x, int a, int b)
{
    if(x<a) x = a;
    if(x>b) x = b;
    return x;
}

/*
�������ƣ�int16 limit(int16 x, int16 y)
����˵������x,y�е���Сֵ
����˵����
�������أ�������ֵ�е���Сֵ
�޸�ʱ�䣺2025��1��5��
��    ע��
example��  limit( x,  y)
 */
int16 limit1(int16 x, int16 y)
{
    if (x > y)             return y;
    else if (x < -y)       return -y;
    else                return x;
}


/*��������*/
uint8 original_image[image_h][image_w];
uint8 image_thereshold;//ͼ��ָ���ֵ
//------------------------------------------------------------------------------------------------------------------
//  @brief      ���һ���Ҷ�ͼ��
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
void Get_image(uint8(*mt9v03x_image)[image_w])
{
#define use_num     1   //1���ǲ�ѹ����2����ѹ��һ��
    uint8 i = 0, j = 0, row = 0, line = 0;
    for (i = 0; i < image_h; i += use_num)          //
    {
        for (j = 0; j <image_w; j += use_num)     //
        {
            original_image[row][line] = mt9v03x_image[i][j];//����Ĳ�����д�������ͷ�ɼ�����ͼ��
            line++;
        }
        line = 0;
        row++;
    }
}
//------------------------------------------------------------------------------------------------------------------
//  @brief     ��̬��ֵ
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 otsuThreshold(uint8 *image, uint16 col, uint16 row)
{
#define GrayScale 256
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
    uint8 Threshold = 0;
    
    
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
              Threshold = (uint8)Y;
          }
    }
   return Threshold;
}
//------------------------------------------------------------------------------------------------------------------
//  @brief      ͼ���ֵ���������õ��Ǵ�򷨶�ֵ����
//  @since      v1.0 
//------------------------------------------------------------------------------------------------------------------
uint8 bin_image[image_h][image_w];//ͼ������
void turn_to_bin(void)
{
  uint8 i,j;
 image_thereshold = otsuThreshold(original_image[0], image_w, image_h);
  for(i = 0;i<image_h;i++)
  {
      for(j = 0;j<image_w;j++)
      {
          if(original_image[i][j]>image_thereshold)bin_image[i][j] = white_pixel;
          else bin_image[i][j] = black_pixel;
      }
  }
}


/*
�������ƣ�void get_start_point(uint8 start_row)
����˵����Ѱ�������߽�ı߽����Ϊ������ѭ������ʼ��
����˵����������������
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example��  get_start_point(image_h-2)
 */
uint8 start_point_l[2] = { 0 };//�������x��yֵ
uint8 start_point_r[2] = { 0 };//�ұ�����x��yֵ
uint8 get_start_point(uint8 start_row)
{
    uint8 i = 0,l_found = 0,r_found = 0;
    //����
    start_point_l[0] = 0;//x
    start_point_l[1] = 0;//y

    start_point_r[0] = 0;//x
    start_point_r[1] = 0;//y

        //���м�����ߣ��������
    for (i = image_w / 2; i > border_min; i--)
    {
        start_point_l[0] = i;//x
        start_point_l[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
        {
            //printf("�ҵ�������image[%d][%d]\n", start_row,i);
            l_found = 1;
            break;
        }
    }

    for (i = image_w / 2; i < border_max; i++)
    {
        start_point_r[0] = i;//x
        start_point_r[1] = start_row;//y
        if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
        {
            //printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
            r_found = 1;
            break;
        }
    }

    if(l_found&&r_found)return 1;
    else if(l_found == 0 && r_found)
    {
        for (i = image_w * 3 / 4; i > border_min; i--)
        {
            start_point_l[0] = i;//x
            start_point_l[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i - 1] == 0)
            {
                //printf("�ҵ�������image[%d][%d]\n", start_row,i);
                l_found = 1;
                break;
            }
        }
        if(l_found&&r_found)return 1;
        else return 0;
    }
    else if(l_found && r_found == 0)
    {
        for (i = image_w / 4; i < border_max; i++)
        {
            start_point_r[0] = i;//x
            start_point_r[1] = start_row;//y
            if (bin_image[start_row][i] == 255 && bin_image[start_row][i + 1] == 0)
            {
                //printf("�ҵ��ұ����image[%d][%d]\n",start_row, i);
                r_found = 1;
                break;
            }
        }
        if(l_found&&r_found)return 1;
        else return 0;
    }
    else {
        //printf("δ�ҵ����\n");
        return 0;
    }
}

/*
�������ƣ�void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

����˵������������ʽ��ʼ���ұߵ�ĺ�������������е�࣬���õ�ʱ��Ҫ©�ˣ������������һ�������ꡣ
����˵����
break_flag_r            �������Ҫѭ���Ĵ���
(*image)[image_w]       ����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
                       �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
*l_stastic              ��ͳ��������ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
*r_stastic              ��ͳ���ұ����ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
l_start_x               �������������
l_start_y               ��������������
r_start_x               ���ұ���������
r_start_y               ���ұ����������
hightest                ��ѭ���������õ�����߸߶�
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example��
    search_l_r((uint16)USE_num,image,&data_stastics_l, &data_stastics_r,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest);
 */

 //��ŵ��x��y����
uint16 points_l[(uint16)USE_num][2] = { {  0 } };//����
uint16 points_r[(uint16)USE_num][2] = { {  0 } };//����
uint16 dir_r[(uint16)USE_num] = { 0 };//�����洢�ұ���������
uint16 dir_l[(uint16)USE_num] = { 0 };//�����洢�����������
uint16 data_stastics_l = 0;//ͳ������ҵ���ĸ���
uint16 data_stastics_r = 0;//ͳ���ұ��ҵ���ĸ���
uint8 hightest = 0;//��ߵ�
void search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest)
{

    uint8 i = 0, j = 0;

    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//ͳ�����
    //����˸�����
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�����˳ʱ��

    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//���������
    uint8 index_r = 0;//�����±�
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//ͳ���ұ�
    //����˸�����
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�������ʱ��

    l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

    //��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //��������ѭ��
    while (break_flag--)
    {

        //���
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l[l_data_statics][0] = center_point_l[0];//x
        points_l[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//������һ

        //�ұ�
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r[r_data_statics][0] = center_point_r[0];//x
        points_r[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//�����㣬��ʹ��
            temp_l[i][1] = 0;//�����㣬��ʹ��
        }

        //����ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l[l_data_statics - 1] = (i);//��¼��������
            }

            if (index_l)
            {
                //���������
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r[r_data_statics][0]== points_r[r_data_statics-1][0]&& points_r[r_data_statics][0] == points_r[r_data_statics - 2][0]
            && points_r[r_data_statics][1] == points_r[r_data_statics - 1][1] && points_r[r_data_statics][1] == points_r[r_data_statics - 2][1])
            ||(points_l[l_data_statics-1][0] == points_l[l_data_statics - 2][0] && points_l[l_data_statics-1][0] == points_l[l_data_statics - 3][0]
                && points_l[l_data_statics-1][1] == points_l[l_data_statics - 2][1] && points_l[l_data_statics-1][1] == points_l[l_data_statics - 3][1]))
        {
            //printf("���ν���ͬһ���㣬�˳�\n");
            break;
        }
        if (my_abs(points_r[r_data_statics][0] - points_l[l_data_statics - 1][0]) < 2
            && my_abs(points_r[r_data_statics][1] - points_l[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n���������˳�\n");
            *hightest = (points_r[r_data_statics][1] + points_l[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
            //printf("\n��y=%d���˳�\n",*hightest);
            break;
        }
        if ((points_r[r_data_statics][1] < points_l[l_data_statics - 1][1]))
        {
//            printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
            continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
        }
        if (dir_l[l_data_statics - 1] == 7
            && (points_r[r_data_statics][1] > points_l[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
        {
            //printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
            center_point_l[0] = (uint8)points_l[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//������һ

        index_r = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//�����㣬��ʹ��
            temp_r[i][1] = 0;//�����㣬��ʹ��
        }

        //�ұ��ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//������һ
                dir_r[r_data_statics - 1] = (i);//��¼��������
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //���������
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //ȡ��ѭ������
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}
/*
�������ƣ�void get_left(uint16 total_L)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_L ���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example�� get_left(data_stastics_l );
 */
uint8 l_border[image_h];//��������
uint8 r_border[image_h];//��������
uint16 l_index[image_h];
uint16 r_index[image_h];//ӳ���ϵ,ӳ����ԭ�����ϵ�λ��
uint8 center_line[image_h];//��������
// ֻ��X�߽�
uint8 x1_boundary[MT9V03X_H], x2_boundary[MT9V03X_H], x3_boundary[MT9V03X_H];

void get_left(uint16 total_L)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    //��ʼ��
    for (i = 0;i<image_h;i++)
    {
        l_border[i] = border_min;
        l_index[i] = 0;
    }
    h = image_h - 2;
    //���
    for (j = 0; j < total_L; j++)
    {
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l[j][0]+1;
            l_index[h] = j;
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0) 
        {
            break;//�����һ���˳�
        }
    }
}
/*
�������ƣ�void get_right(uint16 total_R)
����˵�����Ӱ�����߽�����ȡ��Ҫ�ı���
����˵����
total_R  ���ҵ��ĵ������
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example��get_right(data_stastics_r);
 */
void get_right(uint16 total_R)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h = 0;
    for (i = 0; i < image_h; i++)
    {
        r_index[i] = 0;
        r_border[i] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    h = image_h - 2;
    //�ұ�
    for (j = 0; j < total_R; j++)
    {
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r[j][0] - 1;
            r_index[h] = j;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
        if (h == 0)break;//�����һ���˳�
    }
}


//�������ͺ͸�ʴ����ֵ����
#define threshold_max   255*5//�˲����ɸ����Լ����������
#define threshold_min   255*2//�˲����ɸ����Լ����������
void image_filter(uint8(*bin_image)[image_w])//��̬ѧ�˲�������˵�������ͺ͸�ʴ��˼��
{
    uint16 i, j;
    uint32 num = 0;


    for (i = 1; i < image_h - 1; i++)
    {
        for (j = 1; j < (image_w - 1); j++)
        {
            //ͳ�ư˸����������ֵ
            num =
                bin_image[i - 1][j - 1] + bin_image[i - 1][j] + bin_image[i - 1][j + 1]
                + bin_image[i][j - 1] + bin_image[i][j + 1]
                + bin_image[i + 1][j - 1] + bin_image[i + 1][j] + bin_image[i + 1][j + 1];


            if (num >= threshold_max && bin_image[i][j] == 0)
            {

                bin_image[i][j] = 255;//��  ���Ը�ɺ궨�壬�������

            }
            if (num <= threshold_min && bin_image[i][j] == 255)
            {

                bin_image[i][j] = 0;//��

            }

        }
    }

}

/*
�������ƣ�void image_draw_rectan(uint8(*image)[image_w])
����˵������ͼ��һ���ڿ�
����˵����uint8(*image)[image_w] ͼ���׵�ַ
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example�� image_draw_rectan(bin_image);
 */
void image_draw_rectan(uint8(*image)[image_w])
{

    uint8 i = 0;
    for (i = 0; i < image_h; i++)
    {
        image[i][0] = 0;
        image[i][1] = 0;
        image[i][image_w - 1] = 0;
        image[i][image_w - 2] = 0;

    }
    for (i = 0; i < image_w; i++)
    {
        image[0][i] = 0;
        image[1][i] = 0;
        //image[image_h-1][i] = 0;

    }
}
/*����Ѳ�ߺ���
�������ƣ�void search_l_r(uint16 break_flag, uint8(*image)[image_w],uint16 *l_stastic, uint16 *r_stastic,
                            uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y,uint8*hightest)

����˵������������ʽ��ʼ���ұߵ�ĺ�������������е�࣬���õ�ʱ��Ҫ©�ˣ������������һ�������ꡣ
����˵����
break_flag_r            �������Ҫѭ���Ĵ���
(*image)[image_w]       ����Ҫ�����ҵ��ͼ�����飬�����Ƕ�ֵͼ,�����������Ƽ���
                       �ر�ע�⣬��Ҫ�ú궨��������Ϊ����������������ݿ����޷����ݹ���
*l_stastic              ��ͳ��������ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
*r_stastic              ��ͳ���ұ����ݣ����������ʼ�����Ա����ź�ȡ��ѭ������
l_start_x               �������������
l_start_y               ��������������
r_start_x               ���ұ���������
r_start_y               ���ұ����������
hightest                ��ѭ���������õ�����߸߶�
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example��
    fill_search_l_r((uint16)USE_num,image,&data_stastics_l_fill, &data_stastics_r_fill,start_point_l[0],
                start_point_l[1], start_point_r[0], start_point_r[1],&hightest_fill);
 */

 //��ŵ��x��y����
uint16 points_l_fill[(uint16)USE_num][2] = { {  0 } };//���߲���
uint16 points_r_fill[(uint16)USE_num][2] = { {  0 } };//���߲���
uint16 dir_r_fill[(uint16)USE_num] = { 0 };//�����洢�ұ߲�����������
uint16 dir_l_fill[(uint16)USE_num] = { 0 };//�����洢��߲�����������
uint16 data_stastics_l_fill = 0;//ͳ����߲����ҵ���ĸ���
uint16 data_stastics_r_fill = 0;//ͳ���ұ߲����ҵ���ĸ���
uint8 hightest_fill = 0;//������ߵ�
void fill_search_l_r(uint16 break_flag, uint8(*image)[image_w], uint16 *l_stastic, uint16 *r_stastic, uint8 l_start_x, uint8 l_start_y, uint8 r_start_x, uint8 r_start_y, uint8*hightest_fill)
{

    uint8 i = 0, j = 0;

    //��߱���
    uint8 search_filds_l[8][2] = { {  0 } };
    uint8 index_l = 0;
    uint8 temp_l[8][2] = { {  0 } };
    uint8 center_point_l[2] = {  0 };
    uint16 l_data_statics;//ͳ�����
    //����˸�����
    static int8 seeds_l[8][2] = { {0,  1},{-1,1},{-1,0},{-1,-1},{0,-1},{1,-1},{1,  0},{1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�����˳ʱ��

    //�ұ߱���
    uint8 search_filds_r[8][2] = { {  0 } };
    uint8 center_point_r[2] = { 0 };//���������
    uint8 index_r = 0;//�����±�
    uint8 temp_r[8][2] = { {  0 } };
    uint16 r_data_statics;//ͳ���ұ�
    //����˸�����
    static int8 seeds_r[8][2] = { {0,  1},{1,1},{1,0}, {1,-1},{0,-1},{-1,-1}, {-1,  0},{-1, 1}, };
    //{-1,-1},{0,-1},{+1,-1},
    //{-1, 0},       {+1, 0},
    //{-1,+1},{0,+1},{+1,+1},
    //�������ʱ��

    l_data_statics = *l_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������
    r_data_statics = *r_stastic;//ͳ���ҵ��˶��ٸ��㣬��������ѵ�ȫ��������

    //��һ�θ��������  ���ҵ������ֵ������
    center_point_l[0] = l_start_x;//x
    center_point_l[1] = l_start_y;//y
    center_point_r[0] = r_start_x;//x
    center_point_r[1] = r_start_y;//y

        //��������ѭ��
    while (break_flag--)
    {

        //���
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_l[i][0] = center_point_l[0] + seeds_l[i][0];//x
            search_filds_l[i][1] = center_point_l[1] + seeds_l[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_l_fill[l_data_statics][0] = center_point_l[0];//x
        points_l_fill[l_data_statics][1] = center_point_l[1];//y
        l_data_statics++;//������һ

        //�ұ�
        for (i = 0; i < 8; i++)//����8F����
        {
            search_filds_r[i][0] = center_point_r[0] + seeds_r[i][0];//x
            search_filds_r[i][1] = center_point_r[1] + seeds_r[i][1];//y
        }
        //�����������䵽�Ѿ��ҵ��ĵ���
        points_r_fill[r_data_statics][0] = center_point_r[0];//x
        points_r_fill[r_data_statics][1] = center_point_r[1];//y

        index_l = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_l[i][0] = 0;//�����㣬��ʹ��
            temp_l[i][1] = 0;//�����㣬��ʹ��
        }

        //����ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_l[i][1]][search_filds_l[i][0]] == 0
                && image[search_filds_l[(i + 1) & 7][1]][search_filds_l[(i + 1) & 7][0]] == 255)
            {
                temp_l[index_l][0] = search_filds_l[(i)][0];
                temp_l[index_l][1] = search_filds_l[(i)][1];
                index_l++;
                dir_l_fill[l_data_statics - 1] = (i);//��¼��������
            }

            if (index_l)
            {
                //���������
                center_point_l[0] = temp_l[0][0];//x
                center_point_l[1] = temp_l[0][1];//y
                for (j = 0; j < index_l; j++)
                {
                    if (center_point_l[1] > temp_l[j][1])
                    {
                        center_point_l[0] = temp_l[j][0];//x
                        center_point_l[1] = temp_l[j][1];//y
                    }
                }
            }

        }
        if ((points_r_fill[r_data_statics][0]== points_r_fill[r_data_statics-1][0]&& points_r_fill[r_data_statics][0] == points_r_fill[r_data_statics - 2][0]
            && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 1][1] && points_r_fill[r_data_statics][1] == points_r_fill[r_data_statics - 2][1])
            ||(points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 2][0] && points_l_fill[l_data_statics-1][0] == points_l_fill[l_data_statics - 3][0]
                && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 2][1] && points_l_fill[l_data_statics-1][1] == points_l_fill[l_data_statics - 3][1]))
        {
            //printf("���ν���ͬһ���㣬�˳�\n");
            break;
        }
        if (my_abs(points_r_fill[r_data_statics][0] - points_l_fill[l_data_statics - 1][0]) < 2
            && my_abs(points_r_fill[r_data_statics][1] - points_l_fill[l_data_statics - 1][1] < 2)
            )
        {
            //printf("\n���������˳�\n");
            *hightest_fill = (points_r_fill[r_data_statics][1] + points_l_fill[l_data_statics - 1][1]) >> 1;//ȡ����ߵ�
            //printf("\n��y=%d���˳�\n",*hightest);
            break;
        }
        if ((points_r_fill[r_data_statics][1] < points_l_fill[l_data_statics - 1][1]))
        {
//            printf("\n�����߱��ұ߸��ˣ���ߵȴ��ұ�\n");
            continue;//�����߱��ұ߸��ˣ���ߵȴ��ұ�
        }
        if (dir_l_fill[l_data_statics - 1] == 7
            && (points_r_fill[r_data_statics][1] > points_l_fill[l_data_statics - 1][1]))//��߱��ұ߸����Ѿ�����������
        {
            //printf("\n��߿�ʼ�����ˣ��ȴ��ұߣ��ȴ���... \n");
            center_point_l[0] = (uint8)points_l_fill[l_data_statics - 1][0];//x
            center_point_l[1] = (uint8)points_l_fill[l_data_statics - 1][1];//y
            l_data_statics--;
        }
        r_data_statics++;//������һ

        index_r = 0;//�����㣬��ʹ��
        for (i = 0; i < 8; i++)
        {
            temp_r[i][0] = 0;//�����㣬��ʹ��
            temp_r[i][1] = 0;//�����㣬��ʹ��
        }

        //�ұ��ж�
        for (i = 0; i < 8; i++)
        {
            if (image[search_filds_r[i][1]][search_filds_r[i][0]] == 0
                && image[search_filds_r[(i + 1) & 7][1]][search_filds_r[(i + 1) & 7][0]] == 255)
            {
                temp_r[index_r][0] = search_filds_r[(i)][0];
                temp_r[index_r][1] = search_filds_r[(i)][1];
                index_r++;//������һ
                dir_r_fill[r_data_statics - 1] = (i);//��¼��������
                //printf("dir[%d]:%d\n", r_data_statics - 1, dir_r[r_data_statics - 1]);
            }
            if (index_r)
            {

                //���������
                center_point_r[0] = temp_r[0][0];//x
                center_point_r[1] = temp_r[0][1];//y
                for (j = 0; j < index_r; j++)
                {
                    if (center_point_r[1] > temp_r[j][1])
                    {
                        center_point_r[0] = temp_r[j][0];//x
                        center_point_r[1] = temp_r[j][1];//y
                    }
                }

            }
        }


    }


    //ȡ��ѭ������
    *l_stastic = l_data_statics;
    *r_stastic = r_data_statics;

}
/*
�������ƣ�void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
����˵�����Ӱ������߽߱�����ȡ��Ҫ�ı���
����˵����
total_L ������ҵ��ĵ������
total_R ���ұ��ҵ��ĵ������
h        ����ʼ����
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example�� get_direction_fill(data_stastics_l_fill, data_stastics_r_fill, h);
 */

void get_direction_fill(uint16 total_L, uint16 total_R, uint8 h)
{
    uint8 i = 0;
    uint16 j = 0;
    uint8 h_if = 0;
    //ȷ����ʼ����Χ
    if(h>image_h - 3)
    {
        h_if = image_h;
        h=image_h - 2;
    }
    else if(h<=0)
    {
        h_if = 0;
        h=0;
    }
    else
    {
        h_if = h;
    }
    //��ʼ��
    for (i = 0;i<h_if;i++)
    {
        l_border[i] = border_min;
    }
    for (i = 0; i < h_if; i++)
    {
        r_border[i] = border_max;//�ұ��߳�ʼ���ŵ����ұߣ�����߷ŵ�����ߣ�����������պ�����������߾ͻ����м䣬������ŵõ�������
    }
    //���
    for (j = 0; j < total_L; j++)
    {
        if (h == 0)
        {
            break;//�����һ���˳�
        }
        //printf("%d\n", j);
        if (points_l[j][1] == h)
        {
            l_border[h] = points_l_fill[j][0]+1;
        }
        else continue; //ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
    }
    //�ұ�
    for (j = 0; j < total_R; j++)
    {
        if (h == 0)
        {
            break;//�����һ���˳�
        }
        if (points_r[j][1] == h)
        {
            r_border[h] = points_r_fill[j][0] - 1;
        }
        else continue;//ÿ��ֻȡһ���㣬û����һ�оͲ���¼
        h--;
    }
}

/**
* @brief ��С���˷�
* @param uint8 begin                �������
* @param uint8 end                  �����յ�
* @param uint8 *border              ������Ҫ����б�ʵı߽��׵�ַ
*  @see CTest       Slope_Calculate(start, end, border);//б��
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
float Slope_Calculate(uint8 begin, uint8 end, uint8 *border)
{
    float xsum = 0, ysum = 0, xysum = 0, x2sum = 0;
    int16 i = 0;
    float result = 0;
    static float resultlast;

    for (i = begin; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        xysum += i * (border[i]);
        x2sum += i * i;

    }
    if ((end - begin)*x2sum - xsum * xsum) //�жϳ����Ƿ�Ϊ��
    {
        result = ((end - begin)*xysum - xsum * ysum) / ((end - begin)*x2sum - xsum * xsum);
        resultlast = result;
    }
    else
    {
        result = resultlast;
    }
    return result;
}

/**
* @brief ����б�ʽؾ�
* @param uint8 start                �������
* @param uint8 end                  �����յ�
* @param uint8 *border              ������Ҫ����б�ʵı߽�
* @param float *slope_rate          ����б�ʵ�ַ
* @param float *intercept           ����ؾ��ַ
*  @see CTest       calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
*/
void calculate_s_i(uint8 start, uint8 end, uint8 *border, float *slope_rate, float *intercept)
{
    uint16 i, num = 0;
    uint16 xsum = 0, ysum = 0;
    float y_average, x_average;

    num = 0;
    xsum = 0;
    ysum = 0;
    y_average = 0;
    x_average = 0;
    for (i = start; i < end; i++)
    {
        xsum += i;
        ysum += border[i];
        num++;
    }

    //�������ƽ����
    if (num)
    {
        x_average = (float)(xsum / num);
        y_average = (float)(ysum / num);

    }

    /*����б��*/
    *slope_rate = Slope_Calculate(start, end, border);//б��
    *intercept = y_average - (*slope_rate)*x_average;//�ؾ�
}

/**
 * @brief ��߶��ߺ���
 * @param uint8 *l_border     ��������ߵ�ַ
 * @param uint8 start         ������ʼ��
 * @param uint8 end           ������ֹ��
 * @see CTest       l_loss_judge(l_border,20 ,0)
 * @return ����˵��
 * -1 ��Ч
 * 1  ����
 * 0  ������
 */
int l_loss_judge(uint8 *l_border,uint8 start ,uint8 end)
{
    uint16 i;
    uint16 sum = 0;
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    if(start >= end)
        return -1;
    for(i = start;i <= end; i++)
    {
       if(l_border[i] <= border_min+2 )
           sum++;
    }
    if(sum >= (my_abs(start - end)/5*4))
        return 1;
    else
        return 0;
}

/**
 * @brief �ұ߶��ߺ���
 * @param uint8 *r_border     ��������ߵ�ַ
 * @param uint8 start         ������ʼ��
 * @param uint8 end           ������ֹ��
 * @see CTest       r_loss_judge(l_border,20 ,0)
 * @return ����˵��
 * -1 ��Ч
 * 1  ����
 * 0  ������
 */
int r_loss_judge(uint8 *r_border,uint8 start ,uint8 end)
{
    uint16 i;
    uint16 sum = 0;
    start = (uint8)limit_a_b(start, 2, image_h-2);
    end = (uint8)limit_a_b(end, 2, image_h-2);
    if(start >= end)
        return -1;
    for(i = start;i <= end; i++)
    {
       if(r_border[i] >= border_max-2 )
           sum++;
    }
    if(sum >= (my_abs(start - end)/5*4))
        return 1;
    else
        return 0;
}

uint8 broken_line_x = 0;
uint8 broken_line_y = 0;
/**
 * @brief ����Ѱ�Һ���
 * @param uint8 dir                  ����0:��������Ѱ��1����������Ѱ
 * @param uint8 start                ������ʼ��
 * @param uint8 *border              ����߽��׵�ַ
 * @see CTest       broken_line_judge(1,90,120,l_border);
 * @return ����˵��
 */
void broken_line_judge(int dir, uint8 start, uint8 end, uint8 *border)
{
    if(dir == 0)
    {
        for(uint16 i = (uint16)start ;i <= (uint16)end; i++)
        {
            if(my_abs(border[i] - border[i-1]) >= 4)
            {
                broken_line_x = border[i-1];
                broken_line_y = (uint8)(i-1);
                break;
            }
            if(i == (uint16)(end-2))
            {
                broken_line_x = -1;
                broken_line_y = -1;
                break;
            }
        }
    }
    if(dir == 1)
    {
        for(uint16 i = (uint16)end;i >= (uint16)start;i--)
        {
            if(my_abs(border[i] - border[i+1]) >= 4)
            {
                broken_line_x = border[i+1];
                broken_line_y = (uint8)(i+1);
                break;
            }
            if(i == (int)(start+2))
            {
                broken_line_x = -1;
                broken_line_y = -1;
                break;
            }
        }
    }

}
/**
 * @brief �����жϺ���
 * @param uint8* hightest            ��������߶�
 * @param uint8 *l_border            ������߽��׵�ַ
 * @param uint8 *r_border            �����ұ߽��׵�ַ
 * @param uint16 total_num_l         �������ѭ���ܴ���
 * @param uint16 total_num_r         �����ұ�ѭ���ܴ���
 * @param uint16 *dir_l              ����������������׵�ַ
 * @param uint16 *dir_r              �����ұ����������׵�ַ
 * @param uint16(*points_l)[2]       ������������׵�ַ
 * @param uint16(*points_r)[2]       �����ұ������׵�ַ
 * @param uint16 *l_index            �������ӳ���׵�ַ
 * @param uint16 *r_index            �����ұ�ӳ���׵�ַ
 * @see CTest      monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
 * @return ����˵��
 * 1 ����
 * 0 ������
 */
int monotonicity_l = -1;
int monotonicity_r = -1;
void monotonicity_line(uint8* hightest, uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
        uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],uint16 *l_index,uint16 *r_index)
{
    uint16 i;
    uint16 total_l = 0;
    uint16 total_r = 0;//���������ܼ�
    uint16 arr_l = 0;
    uint16 arr_r = 0;//�������������ܼ�
    uint16 abb_l = 0;
    uint16 abb_r = 0;//���߸����ܼ�
    uint16 acc_l = 0;
    uint16 acc_r = 0;//ӳ��ϵ��ܼ�
    //��ߵ�����
    for (i = 1; i < total_num_l-20; i++)
    {
        if(points_l[i][0] == (uint16)l_border[points_l[i][1]] && points_l[i][1] <= 20)
            break;
        /*ǰ��ʮ�п��ܳ��ַ���Ĳ�ȷ��������*/
        if ((dir_l[i] == 4) || (dir_l[i] == 5) || (dir_l[i] == 6))
        {
            arr_l++;
        }
        total_l++;
    }
    /*�����Ե�һ���������������ж��������÷������Ϊ��������*/
    if(arr_l>=(total_l-5))
    {
        /*ǰ��ʮ�п��ܳ��ַ���Ĳ�ȷ��������*/
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*�����Եڶ�������ӳ��ϵ㣨�ж��������ù���Ϊ��������*/
                }
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*�����Ե����������ϵ㣨�ж�ʮ�ֵȶϵ�Ϊ��������*/
                }
                if(i == 100)
                    /*��ʮ�п��ܳ�����Ӱ�Ĳ�ȷ��������*/
                {
                    monotonicity_l = 1;
                    break;
                }
            }
        }
        else
            for (int i = 20; i < image_h-2; i++)
            {
                if(my_abs(l_index[i]-l_index[i+1]) >= 5)
                    acc_l++;
                if(acc_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*�����Եڶ�������ӳ��ϵ㣨�ж��������ù���Ϊ��������*/
                }
                if(my_abs(l_border[i]-l_border[i+1]) >= 2)
                    abb_l++;
                if(abb_l >= 5)
                {
                    monotonicity_l = 0;
                    break;
                    /*�����Ե����������ϵ㣨�ж�ʮ�ֵȶϵ�Ϊ��������*/
                }
                if(i == 100)
                    /*��ʮ�п��ܳ�����Ӱ�Ĳ�ȷ��������*/
                {
                    monotonicity_l = 1;
                    break;
                }
            }
    }
    else monotonicity_l = 0;
    //�ұߵ�����
    for (i = 1; i < total_num_r-20; i++)
    {
        if(points_r[i][0] == (uint16)r_border[points_r[i][1]] && points_r[i][1] <= 20)
            break;
        /*ǰ��ʮ�п��ܳ��ַ���Ĳ�ȷ��������*/
        if ((dir_r[i] == 4) || (dir_r[i] == 5) || (dir_r[i] == 6))
        {
            arr_r++;
        }
        total_r++;
    }
    /*�����Ե�һ���������������ж��������÷������Ϊ��������*/
    if(arr_r>=(total_r-5))
    {
        /*ǰ��ʮ�п��ܳ��ַ���Ĳ�ȷ��������*/
        if(*hightest>=20)
        {
            for (int i = *hightest; i < image_h-2; i++)
            {
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*�����Եڶ�������ӳ��ϵ㣨�ж��������ù���Ϊ��������*/
                }
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*�����Ե����������ϵ㣨�ж�ʮ�ֵȶϵ�Ϊ��������*/
                }
                if(i == 100)
                    /*��ʮ�п��ܳ�����Ӱ�Ĳ�ȷ��������*/
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
        else
        {
            for (int i = 20; i < image_h-2; i++)
            {
                if(my_abs(r_index[i]-r_index[i+1]) >= 5)
                    acc_r++;
                if(acc_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*�����Եڶ�������ӳ��ϵ㣨�ж��������ù���Ϊ��������*/
                }
                if(my_abs(r_border[i]-r_border[i+1])>=2)
                    abb_r++;
                if(abb_r >= 5)
                {
                    monotonicity_r = 0;
                    break;
                    /*�����Ե����������ϵ㣨�ж�ʮ�ֵȶϵ�Ϊ��������*/
                }
                if(i == 100)
                    /*��ʮ�п��ܳ�����Ӱ�Ĳ�ȷ��������*/
                {
                    monotonicity_r = 1;
                    break;
                }
            }
        }
    }
    else monotonicity_r = 0;
}
/**
* @brief ʮ��״̬������
* @param uint8(*image)[image_w]     �����ֵͼ��
* @param uint8 *l_border            ������߽��׵�ַ
* @param uint8 *r_border            �����ұ߽��׵�ַ
* @param uint16 total_num_l         �������ѭ���ܴ���
* @param uint16 total_num_r         �����ұ�ѭ���ܴ���
* @param uint16 *dir_l              ����������������׵�ַ
* @param uint16 *dir_r              �����ұ����������׵�ַ
* @param uint16(*points_l)[2]       ������������׵�ַ
* @param uint16(*points_r)[2]       �����ұ������׵�ַ
*  @see CTest       cross_fill(image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                                         uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2])
{
    uint16 i;
    uint8 break_num_l = 0;
    uint8 break_num_r = 0;
    uint8 end_num_l = 0;
    uint8 end_num_r = 0;
    uint8 break_num_l_x = 0;
    uint8 break_num_r_x = 0;
    uint8 start, end;
    float slope_l_rate = 0, intercept_l = 0;
    if(cross_sum == 0)
    {
        if(((l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 1
                && r_loss_judge(r_border, 80 ,100) == 0 && r_loss_judge(r_border, 50 ,70) == 1)
                || (l_loss_judge(l_border,80 ,100) == 0 && l_loss_judge(l_border, 50 ,70) == 1
                && r_loss_judge(r_border, 80 ,100) == 1 && r_loss_judge(r_border, 50 ,70) == 1)
                || (l_loss_judge(l_border,80 ,100) == 1 && l_loss_judge(l_border, 50 ,70) == 1
                && r_loss_judge(r_border, 80 ,100) == 0 && r_loss_judge(r_border, 50 ,70) == 1))
                && sum_island == 0
                )
        {
            cross_sum = 1;
        }
    }
    if(cross_sum == 1)//��ʮ��
    {
        for (i = 1; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*��һ�ж���������������ת��*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*�ڶ��ж�����������*/
            {
                break_num_l = (uint8)points_l[i][1];//����y����
                break;
            }
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /*��һ�ж���������������ת��*/
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
                /*�ڶ��ж�����������*/
            {
                end_num_l = (uint8)points_l[i][1];//����y����
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*��һ�ж���������������ת��*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*�ڶ��ж�����������*/
            {
                break_num_r = (uint8)points_r[i][1];//����y����
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                    /*��һ�ж���������������ת��*/
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                         /*�ڶ��ж�����������*/
            {
                end_num_r = (uint8)points_r[i][1];//����y����
            }
        }
        for (int j = break_num_l; j<=end_num_l;j++)
        {
            for(int h=l_border[j]+1;h<=94;h++)
            {
                //ɨ�赽��ɫ���ͽ��ж�
                if (image[j][h-1] == 0 && image[i][h] == 255)
                {
                    cross_sum = 3;//����ʮ��Բ��
                }
            }
        }
        for (int j = break_num_r; j<=end_num_r;j++)
        {
            for(int h=r_border[j]-1;h>=94;h--)
            {
                //ɨ�赽��ɫ���ͽ��ж�
                if (image[j][h+1] == 0 && image[i][h] == 255)
                {
                    cross_sum = 3;//����ʮ��Բ��
                }
            }
        }
        if ((end_num_l == 0 && end_num_r == 0)||(end_num_l >= 110 && end_num_r >= 110)
                ||(break_num_l && break_num_r == 0 && end_num_l == 0 && end_num_r)
                ||(break_num_r && break_num_l == 0 && end_num_r == 0 && end_num_l)
                )
        {
            cross_sum = 2;
        }
        if (break_num_l && break_num_r == 0 && end_num_l && end_num_r == 0)
        {
            //����б��
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б��k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//�ؾ�b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
        if (break_num_r && break_num_l == 0 && end_num_r && end_num_l == 0)
         {
            //����б��
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б��k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//�ؾ�b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
         }
        if (break_num_l && break_num_r && end_num_l && end_num_r)
        {
            //����б��
            slope_l_rate = (float)((end_num_l+5) - (break_num_l-10)) / (l_border[end_num_l+5] - l_border[break_num_l-10]);//б��k=y/x
            intercept_l = (end_num_l+5) - slope_l_rate*l_border[end_num_l+5];//�ؾ�b=y-kx
            for (i = break_num_l-10; i < end_num_l+5; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            //����б��
            slope_l_rate = (float)((end_num_r+5) - (break_num_r-10)) / (r_border[end_num_r+5] - r_border[break_num_r-10]);//б��k=y/x
            intercept_l = (end_num_r+5) - slope_l_rate*r_border[end_num_r+5];//�ؾ�b=y-kx
            for (i = break_num_r-10; i < end_num_r+5; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }

    }
    if(cross_sum == 2)//��ʮ��
    {
        for (i = 10; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*��һ�ж���������������ת��*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*�ڶ��ж�����������*/
            {
                break_num_l = (uint8)points_l[i][1]-5;//����y����
                break;
            }
        }
        for (i = 10; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*��һ�ж���������������ת��*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*�ڶ��ж�����������*/
            {
                break_num_r = (uint8)points_r[i][1]-5;//����y����
                break;
            }
        }
        if((break_num_l == 0 && break_num_r == 0)||(break_num_l >= 115 && break_num_r >= 115)
                ||(l_loss_judge(l_border,90 ,110) == 0 && r_loss_judge(r_border,90 ,110) == 0))
        {
            cross_sum = 0;
        }
        //����б��
        start = break_num_l - 10;
        start = (uint8)limit_a_b(start, 0, image_h);
        end = break_num_l - 5;
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
        for (i = break_num_l - 5; i < image_h - 1; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//�޷�
        }

        //����б��
        start = break_num_r - 10;//���
        start = (uint8)limit_a_b(start, 0, image_h);//�޷�
        end = break_num_r - 5;//�յ�
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
        for (i = break_num_r - 5; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
    if(cross_sum == 3)//ʮ��Բ��
    {
        for (i = 1; i < total_num_l; i++)
        {
            if ((dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] != 4 && dir_l[i + 5] != 4 && dir_l[i + 7] != 4)
                 /*��һ�ж���������������ת��*/
                 && (points_l[i - 1][0] <= border_min+2 && points_l[i][0] <= border_min+2))
                /*�ڶ��ж�����������*/
            {
                break_num_l = (uint8)points_l[i][1];//����y����
                break;
            }
            if ((dir_l[i - 1] != 4 && dir_l[i] != 4 && dir_l[i + 3] == 4 && dir_l[i + 5] == 4 && dir_l[i + 7] == 4)
                /*��һ�ж���������������ת��*/
               && (points_l[i + 3][0] <= border_min+2 && points_l[i + 7][0] <= border_min+2))
                /*�ڶ��ж�����������*/
            {
                end_num_l = (uint8)points_l[i][1];//����y����
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if((dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] != 4 && dir_r[i + 5] != 4 && dir_r[i + 7] != 4)
                    /*��һ�ж���������������ת��*/
               && (points_r[i - 1][0] >= border_max-2 && points_r[i][0] >= border_max-2))
                    /*�ڶ��ж�����������*/
            {
                break_num_r = (uint8)points_r[i][1];//����y����
                break;
            }
            if((dir_r[i - 1] != 4 && dir_r[i] != 4 && dir_r[i + 3] == 4 && dir_r[i + 5] == 4 && dir_r[i + 7] == 4)
                    /*��һ�ж���������������ת��*/
              && (points_r[i + 3][0] >= border_max-2 && points_r[i + 7][0] >= border_max-2))
                         /*�ڶ��ж�����������*/
            {
                end_num_r = (uint8)points_r[i][1];//����y����
            }
        }
        for (int j = break_num_l; j<=end_num_l;j++)
        {
            for(int h=l_border[j]+1;h<=r_border[j]-1;h++)
            {
                //ɨ�赽��ɫ���ͽ��ж�
                if (image[j][h-1] == 0 && image[i][h] == 255)
                {
                    break_num_l_x = 1;
                }
                else
                    break_num_l_x = 0;
            }
        }
        for (int j = break_num_r; j<=end_num_r;j++)
        {
            for(int h=r_border[j]-1;h>=l_border[j]+1;h--)
            {
                //ɨ�赽��ɫ���ͽ��ж�
                if (image[j][h+1] == 0 && image[i][h] == 255)
                {
                    break_num_r_x = 1;
                }
                else
                    break_num_r_x = 0;
            }
        }
        if(break_num_l_x == 0 && break_num_r_x == 0)
        {
            cross_sum = 1;
        }
        //����б��
        start = end_num_l + 4;
        start = (uint8)limit_a_b(start, 0, image_h);
        end = end_num_l + 8;
        end = (uint8)limit_a_b(end, 0, image_h);
        calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
        for (i = 20; i < image_h - 1; i++)
        {
            l_border[i] = slope_l_rate * (i)+intercept_l;//y = kx+b
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);//�޷�
        }

        //����б��
        start = end_num_r + 4;//���
        start = (uint8)limit_a_b(start, 0, image_h);//�޷�
        end = end_num_r + 8;//�յ�
        end = (uint8)limit_a_b(end, 0, image_h);//�޷�
        calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
        for (i = 20; i < image_h - 1; i++)
        {
            r_border[i] = slope_l_rate * (i)+intercept_l;
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
    }
}

/**
* @brief ����״̬������
* @param uint8(*image)[image_w]     �����ֵͼ��
* @param uint8 *l_border            ������߽��׵�ַ
* @param uint8 *r_border            �����ұ߽��׵�ַ
* @param uint16 total_num_l         �������ѭ���ܴ���
* @param uint16 total_num_r         �����ұ�ѭ���ܴ���
* @param uint16 *dir_l              ����������������׵�ַ
* @param uint16 *dir_r              �����ұ����������׵�ַ
* @param uint16(*points_l)[2]       ������������׵�ַ
* @param uint16(*points_r)[2]       �����ұ������׵�ַ
* @param uint16 *l_index            �������ӳ���׵�ַ
* @param uint16 *r_index            �����ұ�ӳ���׵�ַ
* @param int monotonicity_l         ������ߵ�����
* @param int monotonicity_r         ������ߵ�����
*  @see CTest       around_fill(image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index,monotonicity_l,monotonicity_r);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void around_fill(uint8(*image)[image_w], uint8 *l_border, uint8 *r_border, uint16 total_num_l, uint16 total_num_r,
                 uint16 *dir_l, uint16 *dir_r, uint16(*points_l)[2], uint16(*points_r)[2],
                 uint8* hightest,uint16 *l_index,uint16 *r_index, int monotonicity_l, int monotonicity_r)
{
   uint16 i;
   int black_line_sum = 0;
   int black_sum_1 = 0;
   int black_sum_2 = 0;
   uint8 break_num_l = 0;
   uint8 break_num_r = 0;
   uint8 end_num_l = 0;
   uint8 end_num_r = 0;
   uint8 start, end;
   int ap = 1;
   float slope_l_rate = 0, intercept_l = 0;
   //ʶ���Ƿ��л���
   //�󻷵�
   if(monotonicity_l == 0 && monotonicity_r == 1 && sum_island == 0 && island == 0)
       //������һ����:����
   {
       broken_line_judge(1,*hightest,110,l_border);
//       for (uint16 w = broken_line_y + 15; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
//       {
//           if (r_border[w] > r_border[w + 10] && (r_border[w] != border_max || r_border[w + 10] != border_max))
//           {
//                    ap = 0;
//                    break;
//           }
//       }
       for (i = broken_line_y+5; i > *hightest; i--)
       {
           for (int x = (int)l_border[i]; x <= (int)r_border[i]; x++)
           {
               //ɨ�赽��ɫ���ͽ��ж�
               if (image[i][x] == 0 && image[i][x-1] == 255)
               {
                   black_sum_1 = 1;
               }
               //ɨ�赽��ɫ���ͽ��ж�
               if (black_sum_1 == 1 && image[i][x-1] == 0 && image[i][x] == 255)
               {
                   black_sum_2 = 1;
               }
               //��ɨ�赽��ɫ�����˳�
               if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x-1] == 255)
               {
                   black_sum_1 = 0;
                   black_sum_2 = 0;
                   break;
               }
           }
           if(black_sum_2 == 1)
           {
               black_sum_1 = 0;
               black_sum_2 = 0;
               black_line_sum++;
           }
           if(i==15)//ǰʮ�п��������
               break;
       }
       if(black_line_sum>=5 /*����������:ʶ�𵽻���*/
               && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
               && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*����������:���������ж�*/
               && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
           /*����������:��ʮ�ֺ͵����Ų���ͻ*/
               && image[broken_line_y-2][l_border[broken_line_y]-2] == 255)
           /*����������:���ϰ��ﲻ��ͻ*/
       {
            sum_island = 1;//Ԥ�⻷����־λ
            island = 1;//�󻷵��ж�
            black_line_sum = 0;
       }
   }
   //�һ���
   if(monotonicity_l == 1 && monotonicity_r == 0 && sum_island == 0 && island == 0)
       //������һ����:����
   {
       broken_line_judge(1,*hightest,110,r_border);
//       for (uint16 w = broken_line_y + 15; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
//       {
//           if (l_border[w] < l_border[w + 10] && (l_border[w] != border_min || l_border[w + 10] != border_min))
//           {
//                    ap = 0;
//                    break;
//           }
//       }
       for (i = broken_line_y+5; i > *hightest; i--)
       {
           for (int x = (int)r_border[i]; x >= (int)l_border[i]; x--)
           {
               //ɨ�赽��ɫ���ͽ��ж�
               if (image[i][x] == 0 && image[i][x+1] == 255)
               {
                   black_sum_1 = 1;
               }
               //ɨ�赽��ɫ���ͽ��ж�
               if (black_sum_1 == 1 && image[i][x+1] == 0 && image[i][x] == 255)
               {
                   black_sum_2 = 1;
               }
               //��ɨ�赽��ɫ�����˳�
               if (black_sum_2 == 1 && image[i][x] == 0 && image[i][x+1] == 255)
               {
                   black_sum_1 = 0;
                   black_sum_2 = 0;
                   break;
               }
           }
           if(black_sum_2 == 1)
           {
               black_sum_1 = 0;
               black_sum_2 = 0;
               black_line_sum++;
           }
           if(i==15)//ǰʮ�п��������
               break;
       }
       if(black_line_sum>=5 /*����������:ʶ�𵽻���*/
               && l_loss_judge(l_border, 90 ,110) == 0 && r_loss_judge(r_border, 70 ,90) == 0
               && l_loss_judge(l_border, 70 ,90) == 0  && r_loss_judge(r_border, 90 ,110) == 0
               /*����������:���������ж�*/
               && cross_sum == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
           /*����������:��ʮ�ֺ͵����Ų���ͻ*/
               && image[broken_line_y-2][r_border[broken_line_y]+2] == 255)
           /*����������:���ϰ��ﲻ��ͻ*/
       {
            sum_island = 1;//Ԥ�⻷����־λ
            island = 2;//�һ����ж�
            black_line_sum = 0;
       }
   }
    //�󻷵�
    if(island == 1)
    {
        //��������
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,l_border);
            for (uint16 w = broken_line_y + 10; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
            {
                if(w<=15)
                {
                    continue;
                }
                else if(w-10 == my_auu((broken_line_y - 15),110,*hightest))
                {
                    break;
                }
                else
                {
                    if (r_border[w-10] > r_border[w] && (r_border[w-10] != border_max || r_border[w] != border_max))
                    {
                             ap = 0;
                             break;
                    }
                }
            }
            if(ap == 0)//���ǻ����˳�,��ֹ����ȱ�ڣ���ʮ��
            {
                island = 0;
                sum_island = 0;
            }
            if(broken_line_y >= 15 && ap == 1)
            {
                //����б��(����һ������������)
                 start = broken_line_y+5;//���
                 end = broken_line_y+10;//�յ�
                 calculate_s_i(start, end, l_border, &slope_l_rate, &intercept_l);
                 for (i = 1; i < broken_line_y+1; i++)
                 {
                     l_border[i] = slope_l_rate * (i)+intercept_l;
                     l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                 }
                 if((broken_line_y >= 105)||(l_loss_judge(l_border, 100 ,115) == 1))
                 {
                     for (i = 106; i > 15; i--)
                     {
                         //ͨ��ӳ���ҵ㣬�ҵ�������һ��״̬
                         if(points_l[l_index[i]][0]>points_l[l_index[i-7]][0] && points_l[l_index[i]][0]>points_l[l_index[i+7]][0]
                            && points_l[l_index[i-7]][0] != border_min)
                         {
                             sum_island = 2;
                         }
                     }
                 }
            }
        }
        //���뻷��
        if(sum_island == 2)
        {
            int dp = 0;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            if (!image[image_h - 5][5] && !image[image_h - 3][3])
            {
                dp = 1;
            }
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]
                         &&points_l[j][1] > points_l[j + 5][1]
                         &&points_l[j][1] >= points_l[j - 3][1]
                         &&points_l[j][1] >= points_l[j - 5][1])
                    {
                        vp = h;
                        break;
                    }
                }
            }
            for (i = 25; i < image_h - 15; i++)
            {
                if(l_border[i]>=l_border[i-5] && l_border[i]>=l_border[i+5]
                   && l_border[i]>l_border[i-7] && l_border[i]>l_border[i+7]
                   && l_border[i-5] != border_min && l_border[i+5] != border_min)
                {
                    end_num_l = (uint8)i;
                }
                if(vp && end_num_l >= 60)
                    sum_island = 3;
            }
                //����б�ʣ�˫�����Ӳ��ߣ�
                slope_l_rate = (float)(118-end_num_l) / (border_min-l_border[end_num_l]);//б��k=y/x
                intercept_l = 118 - slope_l_rate*border_min;//�ؾ�b=y-kx
                for (i = end_num_l; i < image_h - 1; i++)
                {
                    l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
                }

        }
        if(sum_island == 3)
        {
            uint16 h = 0;
            int temph_l = 0;
            for (h = image_h - 15; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    temph_l = h;
                    break;
                }
            }
            if (temph_l)
            {
                for (int i = l_index[temph_l - 2]; i > l_index[temph_l + 1]; i--)
                {

                    if (points_l[i][1] >= points_l[i + 3][1]
                        &&points_l[i][1] > points_l[i + 5][1]
                        &&points_l[i][1] > points_l[i + 7][1]
                        &&points_l[i][1] >= points_l[i - 3][1]
                        &&points_l[i][1] >= points_l[i - 5][1]
                        &&points_l[i][1] > points_l[i - 8][1]
                        &&points_l[i][0] > points_l[i - 5][0]
                        &&points_l[i][0] < points_l[i + 5][0])
                    {
                        break_num_l = (uint8)points_l[i][1];//����y����
                        end_num_l  = (uint8)points_l[i][0];//����x����
                        break;
                    }
                }
            }
            if((*hightest >= 30)&&(!image[image_h - 1][image_w - 3] && !image[image_h - 3][image_w - 3]))
            {
                sum_island = 4;
            }
            if(break_num_l && end_num_l)
            {
                //����б��
                slope_l_rate = (float)(break_num_l-118) / (end_num_l-186);//б��k=y/x
                intercept_l = 118 - slope_l_rate*186;//�ؾ�b=y-kx
                for (i = 1; i < image_h - 1; i++)
                {
                    r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                    r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                }
                for (i = 1; i < break_num_l; i++)
                {
                    l_border[i] = border_min;
                }
            }
        }
        if(sum_island == 4)
        {
            int g=0;
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    g=1;
                    break;
                }
            }
            for (i = image_h - 20; i > *hightest; i--)//�����б�
            {
                if (r_border[i] < r_border[i - 3] && my_abs(r_border[i-3] - r_border[i])>10)
                {
                    if(i>=30 && i<=105 && g)
                        sum_island = 5;
                }
            }
        }
        if(sum_island == 5)
        {
            for (uint16 w = image_h - 15; w > *hightest; w--)//�����б�
            {
                if (r_border[w] < r_border[w - 3] && my_abs(r_border[w-3] - r_border[w])>10)
                {
                    break_num_l = (uint8)w;//����y����
                    break;
                }
            }
            for (i = 1; i < total_num_l; i++)
            {
                if (dir_l[i - 1] == 4 && dir_l[i] == 4 && dir_l[i + 3] == 6 && dir_l[i + 5] == 6 && dir_l[i + 7] == 6)
                {
                    end_num_l = (uint8)points_l[i][1];//����y����
                    break;
                }
            }
            if(r_loss_judge(r_border, 95 ,110) == 1 && r_loss_judge(r_border, 80 ,95) == 1)
                sum_island = 6;
            //����б��
            slope_l_rate = (float)(break_num_l-20) / (r_border[break_num_l]-0);//б��k=y/x
            intercept_l = 20 - slope_l_rate*0;//�ؾ�b=y-kx
            for (i = 1; i < break_num_l-3; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
            for (i = 1; i < end_num_l; i++)
            {
                l_border[i] = border_min;
            }
        }
        //��Բ��
        if(sum_island == 6)
        {
            int dp = 1;
            int temph = 0;
            int vp = 0;
            uint16 h = 0;
            if (dp)
            {
                for (h = image_h - 15; h > 5; h--)
                {
                    if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 20)
                    {
                        temph = h;
                        break;
                    }
                }
            }
            if (temph)
            {
                for (int j = l_index[h]; j > 0; j--)
                {
                    if (points_l[j][1] >= points_l[j + 3][1]
                         &&points_l[j][1] > points_l[j + 5][1]
                         &&points_l[j][1] >= points_l[j - 3][1]
                         &&points_l[j][1] >= points_l[j - 5][1])
                    {
                        vp = h;
                        break;
                    }
                }
            }
            if(vp && r_loss_judge(r_border, 70 ,90) == 0
                    && r_loss_judge(r_border, 50 ,70) == 0)
                sum_island = 7;
            //����б��
            slope_l_rate = (float)(118-20) / (186-0);//б��k=y/x
            intercept_l = 20 - slope_l_rate*0;//�ؾ�b=y-kx
            for (i = 1; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
        }
        //������
        if(sum_island == 7)
        {
            uint16 h = 0;
            for (h = image_h - 25; h > 5; h--)
            {
                if (l_border[h] > l_border[h + 1] && my_abs(l_border[h] - l_border[h + 1]) > 10)
                {
                    break_num_l = (uint8)(h-2);//����y����
                    end_num_l  = (uint8)l_border[h - 2];//����x����
                    break;
                }
            }
            if ((!image[image_h - 10][image_w - 10] && !image[image_h - 5][image_w - 5]
                 && !image[image_h - 15][image_w - 15] && !image[image_h - 20][image_w - 15]
                 && !image[image_h - 10][10] && !image[image_h - 5][5]
                 && !image[image_h - 20][15] && !image[image_h - 15][15]) || h >= 110)
            {
                if(r_loss_judge(r_border, 100 ,110) == 0 && l_loss_judge(l_border, 100 ,110) == 0
                      && r_loss_judge(r_border, 90 ,100) == 0 && l_loss_judge(l_border, 90 ,100) == 0
                      && r_loss_judge(r_border, 80 ,90) == 0 && l_loss_judge(l_border, 80 ,90) == 0
                      && r_loss_judge(r_border, 60 ,70) == 0 && l_loss_judge(l_border, 60 ,70) == 0
                      && r_loss_judge(r_border, 70 ,80) == 0 && l_loss_judge(l_border, 70 ,80) == 0)
                {
                    sum_island = 0;
                    island = 0;
                }
            }
            //����б�ʣ�˫�����Ӳ��ߣ�
            slope_l_rate = (float)(118-break_num_l) / (2-l_border[break_num_l]);//б��k=y/x
            intercept_l = 118 - slope_l_rate*2;//�ؾ�b=y-kx
            for (i = break_num_l-10; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
        }
    }
    //�һ���
    if(island == 2)
    {
        //��������
        if(sum_island == 1)
        {
            broken_line_judge(1,*hightest,110,r_border);
            for (uint16 w = broken_line_y + 10; w >= my_auu((broken_line_y - 5),110,*hightest); w--)
            {
                if(w<=15)
                {
                    continue;
                }
                else if(w-10 == my_auu((broken_line_y - 15),110,*hightest))
                {
                    break;
                }
                else
                {
                    if (l_border[w-10] < l_border[w] && (l_border[w-10] != border_min || l_border[w] != border_min))
                    {
                             ap = 0;
                             break;
                    }

                }
            }
            if(ap == 0)//���ǻ����˳�,��ֹ����ȱ�ڣ���ʮ��
             {
                 island = 0;
                 sum_island = 0;
             }
            if(broken_line_y >= 15 && ap == 1)
            {
                //����б��(����һ������������)
                 start = broken_line_y+5;//���
                 end = broken_line_y+10;//�յ�
                 calculate_s_i(start, end, r_border, &slope_l_rate, &intercept_l);
                 for (i = 1; i < broken_line_y+1; i++)
                 {
                     r_border[i] = slope_l_rate * (i)+intercept_l;
                     r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
                 }
                 if((broken_line_y >= 105)||(r_loss_judge(r_border, 100 ,115) == 1))
                 {
                     for (i = 106; i > 15; i--)
                     {
                         //ͨ��ӳ���ҵ㣬�ҵ�������һ��״̬
                         if(points_r[r_index[i]][0]<points_r[r_index[i-7]][0] && points_r[r_index[i]][0]<points_r[r_index[i+7]][0]
                              && points_r[r_index[i-7]][0] != border_max)
                         {
                             sum_island = 2;
                         }
                     }
                 }
            }
        }
    //���뻷��
    if(sum_island == 2)
    {
        int dp = 0;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (!image[image_h - 5][image_w - 5] && !image[image_h - 3][image_w - 3])
        {
            dp = 1;
        }
        if (dp)
        {
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
                {
                    temph = h;
                    break;
                }
            }
        }
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
        for (i = 25; i < image_h - 15; i++)
        {
            if(r_border[i]<=r_border[i-5] && r_border[i]<=r_border[i+5]
               && r_border[i]<r_border[i-7] && r_border[i]<r_border[i+7]
               && r_border[i-7] != border_max && r_border[i+7] != border_max)
            {
                end_num_r = (uint8)i;
            }
            if(vp && end_num_r >= 60)
                sum_island = 3;
        }
            //����б�ʣ�˫�����Ӳ��ߣ�
            slope_l_rate = (float)(118-end_num_r) / (border_max-r_border[end_num_r]);//б��k=y/x
            intercept_l = 118 - slope_l_rate*border_max;//�ؾ�b=y-kx
            for (i = end_num_r-10; i < image_h - 1; i++)
            {
                r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
            }
    }
    if(sum_island == 3)
    {
        uint16 h = 0;
        int temph_r = 0;
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
            {
                temph_r = h;
                break;
            }
        }
        if (temph_r)
        {
            for (int i = r_index[temph_r - 2]; i > r_index[temph_r + 1]; i--)
            {

                if (points_r[i][1] >= points_r[i + 3][1]
                    &&points_r[i][1] > points_r[i + 5][1]
                    &&points_r[i][1] > points_r[i + 7][1]
                    &&points_r[i][1] >= points_r[i - 3][1]
                    &&points_r[i][1] >= points_r[i - 5][1]
                    &&points_r[i][1] > points_r[i - 8][1]
                    &&points_r[i][0] < points_r[i - 5][0]
                    &&points_r[i][0] > points_r[i + 5][0])
                {
                    break_num_r = (uint8)points_r[i][1];//����y����
                    end_num_r  = (uint8)points_r[i][0];//����x����
                    break;
                }
            }
        }
        if((*hightest >= 30)&& (!image[image_h - 1][3] && !image[image_h - 3][3]))
        {
            sum_island = 4;
        }
        if(break_num_r && end_num_r)
        {
            //����б��
            slope_l_rate = (float)(break_num_r-118) / (end_num_r-2);//б��k=y/x
            intercept_l = 118 - slope_l_rate*2;//�ؾ�b=y-kx
            for (i = 1; i < image_h - 1; i++)
            {
                l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
                l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
            }
            for (i = 1; i < break_num_r; i++)
            {
                r_border[i] = border_max;
            }
        }
    }
    if(sum_island == 4)
    {
        int g=0;
        for (i = 1; i < total_num_r; i++)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
            {
                g=1;
                break;
            }
        }
        for (i = image_h - 20; i > *hightest; i--)//�����б�
        {
            if (l_border[i] > l_border[i - 3] && my_abs(l_border[i-3] - l_border[i])>10)
            {
                if(i>=30 && i<=105 && g)
                    sum_island = 5;
            }
        }
    }
    if(sum_island == 5)
    {
        for (uint16 w = image_h - 15; w > *hightest; w--)//�����б�
        {
            if (l_border[w] > l_border[w - 3] && my_abs(l_border[w] - l_border[w-3])>10)
            {
                break_num_r = (uint8)w;//����y����
                break;
            }
        }
        for (i = 1; i < total_num_r; i++)
        {
            if (dir_r[i - 1] == 4 && dir_r[i] == 4 && dir_r[i + 3] == 6 && dir_r[i + 5] == 6 && dir_r[i + 7] == 6)
            {
                end_num_r = (uint8)points_r[i][1];//����y����
                break;
            }
        }
        if(l_loss_judge(l_border, 95 ,110) == 1 && l_loss_judge(l_border, 80 ,95) == 1)
            sum_island = 6;
        //����б��
        slope_l_rate = (float)(break_num_r-20) / (l_border[break_num_r]-188);//б��k=y/x
        intercept_l = 20 - slope_l_rate*188;//�ؾ�b=y-kx
        for (i = 1; i < break_num_r-3; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
        for (i = 1; i < end_num_r; i++)
        {
            r_border[i] = border_max;
        }
    }
    //��Բ��
    if(sum_island == 6)
    {
        int dp = 1;
        int temph = 0;
        int vp = 0;
        uint16 h = 0;
        if (dp)
        {
            for (h = image_h - 15; h > 5; h--)
            {
                if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 20)
                {
                    temph = h;
                    break;
                }
            }
        }
        if (temph)
        {
            for (int j = r_index[h]; j > 0; j--)
            {
                if (points_r[j][1] >= points_r[j + 3][1]
                     &&points_r[j][1] > points_r[j + 5][1]
                     &&points_r[j][1] >= points_r[j - 3][1]
                     &&points_r[j][1] >= points_r[j - 5][1])
                {
                    vp = h;
                    break;
                }
            }
        }
        if(vp && l_loss_judge(l_border, 70 ,90) == 0
                && l_loss_judge(l_border, 50 ,70) == 0)
            sum_island = 7;
        //����б��
        slope_l_rate = (float)(118-20) / (0-188);//б��k=y/x
        intercept_l = 20 - slope_l_rate*188;//�ؾ�b=y-kx
        for (i = 1; i < image_h - 1; i++)
        {
            l_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            l_border[i] = (uint8)limit_a_b(l_border[i], border_min, border_max);
        }
    }
    //������
    if(sum_island == 7)
    {
        uint16 h = 0;
        for (h = image_h - 15; h > 5; h--)
        {
            if (r_border[h] < r_border[h + 1] && my_abs(r_border[h] - r_border[h + 1]) > 10)
            {
                break_num_r = (uint8)(h-2);//����y����
                end_num_r  = (uint8)r_border[h - 2];//����x����
                break;
            }
        }
        if ((!image[image_h - 10][image_w - 10] && !image[image_h - 5][image_w - 5]
             && !image[image_h - 15][image_w - 15] && !image[image_h - 20][image_w - 15]
             && !image[image_h - 10][10] && !image[image_h - 5][5]
             && !image[image_h - 20][15] && !image[image_h - 15][15]) || h >= 110)
        {
            if(r_loss_judge(r_border, 100 ,110) == 0 && l_loss_judge(l_border, 100 ,110) == 0
                  && r_loss_judge(r_border, 90 ,100) == 0 && l_loss_judge(l_border, 90 ,100) == 0
                  && r_loss_judge(r_border, 80 ,90) == 0 && l_loss_judge(l_border, 80 ,90) == 0
                  && r_loss_judge(r_border, 60 ,70) == 0 && l_loss_judge(l_border, 60 ,70) == 0
                  && r_loss_judge(r_border, 70 ,80) == 0 && l_loss_judge(l_border, 70 ,80) == 0)
            {
                sum_island = 0;
                island = 0;
            }
        }
        //����б�ʣ�˫�����Ӳ��ߣ�
        slope_l_rate = (float)(118-break_num_r) / (186-r_border[break_num_r]);//б��k=y/x
        intercept_l = 118 - slope_l_rate*186;//�ؾ�b=y-kx
        for (i = break_num_r-10; i < image_h - 1; i++)
        {
            r_border[i] = ((i)-intercept_l)/slope_l_rate;//x=(y-b)/k
            r_border[i] = (uint8)limit_a_b(r_border[i], border_min, border_max);
        }
     }
   }
}
/**
* @brief ������״̬������
* @param uint8(*image)[image_w]     �����ֵͼ��
* @param uint8 *l_border            ������߽��׵�ַ
* @param uint8 *r_border            �����ұ߽��׵�ַ
*  @see CTest       cross_stop(image,l_border,r_border);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void cross_stop(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border)
{
    uint8 start_point  = 80;
    uint8 end_point = 100;
    //������־λ
    int banmaxian_kuandu;//�����߿��
    int banmaxian_hangshu;//����������
    int banmaxian_geshu;//�����߸������飩
    banmaxian_kuandu=0;
    banmaxian_hangshu=0;
    banmaxian_geshu=0;
    //��������ɨ��
    for (uint16 y = end_point; y >= start_point; y--)
    {
         //��������ɨ��
         for (int x = (int)(l_border[y]+1); x <= (int)(r_border[y]-1); x++)
        {
            int baidian_heng = 0;
            //ɨ�赽��ɫ���ͽ��ж�
            if (image[y][x] == 0 && image[y][x-1] == 255)
            {
                for(int a=x+1;a<x+15;a++)//�Ӻ�ɫ�������ɨ��
                {
                    //�ҵ���ɫ��
                    if(image[y][a-1] == 0 && image[y][a] == 255)
                    {
                        //��¼��ɫ���λ�ã�����ѭ��
                        baidian_heng=a;
                        break;
                    }
                }//�����߿�ȵ��ںڰ׵�Ĳ�
                banmaxian_kuandu = baidian_heng - x;
                //�����ߵĿ����2~6֮����Ϊ������Ϊ�����ߺ�ɫ��
                if (banmaxian_kuandu >= 2 && banmaxian_kuandu <= 10)
                {
                    //�����ߺ�ɫ��++
                    banmaxian_geshu++;
                    //������ɫ�������㣬������һ����ɫ���ɨ�����
                    banmaxian_kuandu = 0;
                }
                //�����ߵĿ�Ȳ���4~8֮�䲻��Ϊ������Ϊ�����ߺ�ɫ��
                else
                {
                    //���������Ժ�ɫ�����ΪҪ���ֱ�����㣬ȥ������һ����ɫ��
                    banmaxian_kuandu = 0;
                }
            }
        }
        //���ɫ��ĸ�����6~14֮������Ϊ��һ�еİ���������Ҫ����ȥɨ��һ��
        if (banmaxian_geshu >= 6 && banmaxian_geshu <= 14)
        {
            banmaxian_hangshu++;
        }
        else
            banmaxian_geshu = 0;
    }
    //����д��ڵ���4�е���Ч������
    if(banmaxian_hangshu >= 4 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
            && jump_position == 0 && sum_island == 0 && island == 0 && cross_sum == 0)
    {
        //�����߱�׼λ��1
        stop_position = 1;
    }
}
/**
* @brief ������״̬������
* @param uint8(*image)[image_w]     �����ֵͼ��
* @param uint8 *l_border            ������߽��׵�ַ
* @param uint8 *r_border            �����ұ߽��׵�ַ
* @param uint8 *center_line         �����б߽��׵�ַ
* @param uint16 total_num_l         �������ѭ���ܴ���
* @param uint16 total_num_r         �����ұ�ѭ���ܴ���
* @param uint16 *dir_l              ����������������׵�ַ
* @param uint16 *dir_r              �����ұ����������׵�ַ
* @param uint16(*points_l)[2]       ������������׵�ַ
* @param uint16(*points_r)[2]       �����ұ������׵�ַ
* @param int8* hightest             ��������߶�
* @param uint16 *l_index            �������ӳ���׵�ַ
* @param uint16 *r_index            �����ұ�ӳ���׵�ַ
*  @see CTest       bridge_fill(bin_image,l_border, r_border, center_line, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void bridge_fill(uint8(*image)[image_w],uint8 *l_border, uint8 *r_border, uint8 *center_line,
        uint16 total_num_l, uint16 total_num_r, uint16 *dir_l, uint16 *dir_r,
        uint16(*points_l)[2], uint16(*points_r)[2],uint8* hightest,uint16 *l_index,uint16 *r_index)
{
    int h = 0;
    int w = 0;
    int white_line1 = 0;
    int white_line2 = 0;
    int blake_line = 0;
    int cross1 = 0;
    int cross2 = 0;//�յ��ж�
    int long_start_l = 0;
    int long_end_l = 0;
    int long_start_r = 0;
    int long_end_r = 0;
    int loss = 0;//�յ�����ж�
    int bridge_number = 0;
    for (int i = 15; i < image_h-10; i++)
    {
        if(my_abs(l_border[i] - l_border[i+3]) >= 6 && l_border[i]>l_border[i+3]
             && l_border[i]<r_border[i] && l_border[i] < 100)
            loss++;//�յ�����ж�
        if(my_abs(r_border[i] - r_border[i+3]) >= 6 && r_border[i]<r_border[i+3]
            && l_border[i]<r_border[i] && r_border[i] > 80 )
            loss++;
    }
    if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE)
    {
        if(*hightest <= 15)//ֱ���ж�
        {
            for (int i = image_h-20; i>=15 ;i--)
            {
                for (int x = (l_border[i]+1); x <= (r_border[i]-1); x++)
                {
                    if(image[i][x] == 0)
                    {
                        blake_line++;
                        break;
                    }
                    if(image[i][x] == 255)//�ж���
                    {
                        white_line1++;
                    }
                }
                for (int x = (l_border[i-3]+1); x <= (r_border[i-3]-1); x++)
                {
                    if(image[i-3][x] == 0)
                    {
                        blake_line++;
                        break;
                    }
                    if(image[i-3][x] == 255)//��������
                    {
                        white_line2++;
                    }
                }
                if(blake_line >=3)
                {
                    blake_line = 1;
                    break;
                }
                else
                    blake_line = 0;
                if(white_line2 <= ((white_line1-3)/2))
                {
                    h=i-3;
                    for (int j = l_index[h-2]; j > l_index[h+5]; j--)
                    {
                        if (points_l[j][1] >= points_l[j + 1][1]
                            &&points_l[j + 1][0] >= points_l[j][0]
                            &&points_l[j][1] > points_l[j + 2][1]
                            &&points_l[j][0] <= points_l[j + 2][0]
                            &&points_l[j][1] == points_l[j - 3][1]
                            &&points_l[j][0] > points_l[j - 3][0]
                            &&points_l[j][0] > points_l[j - 5][0]
                            &&points_l[j][1] >= points_l[j - 5][1])
                        {
                            cross2=1;//��յ�
                            h=points_l[j][1];
                        }
                    }
                    for (int j = r_index[h-2]; j >= r_index[h+5]; j--)
                    {
                        if (points_r[j][1] >= points_r[j + 1][1]
                            &&points_r[j + 1][0] <= points_r[j][0]
                            &&points_r[j][1] > points_r[j + 2][1]
                            &&points_r[j][0] >= points_r[j + 2][0]
                            &&points_r[j][1] == points_r[j - 3][1]
                            &&points_r[j][0] < points_r[j - 3][0]
                            &&points_r[j][0] < points_r[j - 5][0]
                            &&points_r[j][1] >= points_r[j - 5][1])
                        {
                            cross2=2;//�ҹյ�
                            h=points_r[j][1];
                        }
                    }
                    for (int j = h; j>=15 ;j--)
                    {
                       if(cross2==1 && l_border[j]==l_border[j-1])
                       {
                           for (w = l_index[j] + 5; w >= l_index[j]; w--)
                           {
                               if (dir_l[w] < 4)
                                   break;
                           }
                           if(w <= l_index[j]+1)
                           {
                               cross1++;
                               w=0;
                           }
                           else
                               w=0;
                       }
                       if(cross2==2 && r_border[j]==r_border[j-1])
                       {
                           for (w = r_index[j] + 5; w >= r_index[j]; w--)
                           {
                               if (dir_r[w] < 4)
                                   break;
                           }
                           if(w <= r_index[j]+1)
                           {
                               cross1++;
                               w=0;
                           }
                           else
                               w=0;
                       }
                    }
                    break;
                }
                else
                {
                    h=0;
                    white_line1 = 0;
                    white_line2 = 0;
                }
            }
            if(h>=35 && loss>=6 && blake_line == 0 && cross1>=5 && cross2>0)//60���ǵ�����ʱ���룬�������Ϲյ�
            {
                if(sum_island == 0 && island == 0 && jump_position == 0 && cross_sum == 0
                    &&l_loss_judge(l_border,(uint8)h ,110) == 0 && l_loss_judge(l_border,(uint8)h ,(uint8)(h+20)) == 0
                    && r_loss_judge(r_border,(uint8)h ,110) == 0 && r_loss_judge(r_border,(uint8)h ,(uint8)(h+20)) == 0)
                BridgeState = SINGLE_BRIDGE_ACTIVE;//���뵥���ű�־λ
            }
        }
    }
    if(BridgeState == SINGLE_BRIDGE_ACTIVE)
    {
            for (int i = 30; i < image_h-1; i++)
            {
                if(i == 30)
                {
                    if(my_abs(l_border[i]-94)<=5 && my_abs(r_border[i]-94)>=10)
                    {
                        for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                        {
                            if(a == image_h-5)
                            {
                                BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//���ǵ�����
                                break;
                            }
                            if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                                    image[a+2][l_border[a-3]] == 255 && image[a-3][l_border[a-3]-5] == 0)
                            {
                                long_start_l = i;
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }

                        }
                    }
                }
                if(i>30 && my_abs(l_border[i] - l_border[i+3]) >= 10 && long_start_l == 0 && long_end_l == 0)
                {
                    if(image[i][l_border[i+3]] == 255 && image[i+6][l_border[i+3]-5] == 0)
                    {
                        long_start_l = i+3;
                        for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                        {
                            if(a == image_h-5)
                            {
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }
                            if(my_abs(l_border[a] - l_border[a-3]) >= 10 &&
                                    image[a+2][l_border[a-3]] == 255 && image[a-3][l_border[a-3]-5] == 0)
                            {
                                long_end_l = a;
                                bridge_number++;
                                break;
                            }

                        }
                    }
                }
                if(i <= long_end_l && i >= long_start_l && long_start_l != 0 && long_end_l != 0)
                {
                    center_line[i] = l_border[i];
                    if(i+1 == long_end_l)
                    {
                        long_end_l = 0;
                        long_start_l = 0;
                    }
                }
            }
            for (int i = 30; i < image_h-1; i++)
            {
                if(i == 30)
                {
                    if(my_abs(r_border[i]-94)<=5 && my_abs(l_border[i]-94)>=10)
                    {
                         for(int a = i+1;a < my_auu(i+60,image_h-1,*hightest); a++)
                         {
                             if(a == image_h-5)
                             {
                                 BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//���ǵ�����
                                 break;
                             }
                             if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                                     image[a+2][r_border[a-3]] == 255 && image[a-3][r_border[a-3]+5] == 0)
                             {
                                 long_start_r = i;
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                         }
                    }
                }
                if(i>30 && my_abs(r_border[i] - r_border[i+3]) >= 10 && long_start_r == 0 && long_end_r == 0)
                {
                    if(image[i][r_border[i+3]] == 255 && image[i+6][r_border[i+3]+5] == 0)
                    {
                        long_start_r = i+3;
                         for(int a = i+5;a < my_auu(i+60,image_h-1,*hightest); a++)
                         {
                             if(a == image_h-5)
                             {
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                             if(my_abs(r_border[a] - r_border[a-3]) >= 10 &&
                                     image[a+2][r_border[a-3]] == 255 && image[a-3][r_border[a-3]+5] == 0)
                             {
                                 long_end_r = a;
                                 bridge_number++;
                                 break;
                             }
                         }
                    }
                }
                if(i <= long_end_r && i >= long_start_r && long_start_r != 0 && long_end_r != 0)
                {
                    center_line[i] = r_border[i];
                    if(i+1==long_end_r)
                    {
                        long_end_r = 0;
                        long_start_r = 0;
                    }
                }
            }
            if(bridge_number == 0)
            {
                cross2=0;
                for (int i = image_h-20; i>=50 ;i--)
                {
                    for (int x = (l_border[i]+1); x <= (r_border[i]-1); x++)
                    {
                        if(image[i][x] == 255)//�ж���
                        {
                            white_line1++;
                        }
                    }
                    for (int x = (l_border[i-3]+1); x <= (r_border[i-3]-1); x++)
                    {
                        if(image[i-3][x] == 255)//��������
                        {
                            white_line2++;
                        }
                    }
                    if(white_line2 <= ((white_line1-3)/2))
                    {
                        h=i-3;
                        for (int j = l_index[h-2]; j > l_index[h+5]; j--)
                        {
                            if (points_l[j][1] >= points_l[j + 1][1]
                                &&points_l[j + 1][0] >= points_l[j][0]
                                &&points_l[j][1] > points_l[j + 2][1]
                                &&points_l[j][0] <= points_l[j + 2][0]
                                &&points_l[j][1] == points_l[j - 3][1]
                                &&points_l[j][0] > points_l[j - 3][0]
                                &&points_l[j][0] > points_l[j - 5][0]
                                &&points_l[j][1] >= points_l[j - 5][1])
                            {
                                cross2=1;//��յ�
                                h=points_l[j][1];
                            }
                        }
                        for (int j = r_index[h-2]; j >= r_index[h+5]; j--)
                        {
                            if (points_r[j][1] >= points_r[j + 1][1]
                                &&points_r[j + 1][0] <= points_r[j][0]
                                &&points_r[j][1] > points_r[j + 2][1]
                                &&points_r[j][0] >= points_r[j + 2][0]
                                &&points_r[j][1] == points_r[j - 3][1]
                                &&points_r[j][0] < points_r[j - 3][0]
                                &&points_r[j][0] < points_r[j - 5][0]
                                &&points_r[j][1] >= points_r[j - 5][1])
                            {
                                cross2=2;//�ҹյ�
                                h=points_r[j][1];
                            }
                        }
                        break;
                    }
                    else
                    {
                        white_line1 = 0;
                        white_line2 = 0;
                        h=0;
                    }
                }
                if(cross2 == 0
                   &&l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,60 ,80) == 0
                   && r_loss_judge(r_border,90 ,110) == 0 && r_loss_judge(r_border,60 ,80) == 0)
                BridgeState = SINGLE_BRIDGE_NOT_ACTIVE;//�˳������ű�־λ
            }
        }
    }

/**
* @brief ��Ծ״̬������
* @param uint8(*image)[image_w]     �����ֵͼ��
* @param uint8* hightest            ��������߶�
* @param uint8 *l_border            ������߽��׵�ַ
* @param uint8 *r_border            �����ұ߽��׵�ַ
* @param uint8 monotonicity_l       ������ߵ�����
* @param uint8 monotonicity_r       �����ұߵ�����
* @see CTest       jump_judge(bin_image,&hightest,l_border,r_border,monotonicity_l, monotonicity_r,(Encoder_Left-Encoder_Right)/2);
* @return ����˵��
*     -<em>false</em> fail
*     -<em>true</em> succeed
 */
void jump_judge(uint8(*image)[image_w], uint8* hightest,uint8 *l_border, uint8 *r_border, int monotonicity_l, int monotonicity_r,float now_velocity)
{
    if(jump_position == 0)
    {
        int blake_line = 0;
        jump_h = (int)(-0.1006*now_velocity+103.8);
        if(now_velocity >= 580)
        {
            jump_h = (int)(-0.1056*now_velocity+107.5);
            if(now_velocity >= 700)
            {
                jump_h = jump_h+1;
            }
            else if(now_velocity >= 650)
            {
                jump_h = jump_h-2;
            }
            else if(now_velocity >= 620)
            {
                jump_h = jump_h-1;
            }
        }
        else if(now_velocity > 550 && now_velocity < 580)
        {
            jump_h = jump_h+1;
        }
        else if(now_velocity < 400)
        {
            jump_h = jump_h+2;
        }
        my_auu(jump_h,110,30);
       if(*hightest>=(uint8)jump_h)
       {
           for (int y = (int)*hightest; y >= jump_h-5; y--)
           {
               for (int x = l_border[*hightest+1]; x <= r_border[*hightest+1]; x++)
               {
                   if(image[y][x] == 255)
                   {
                       break;
                   }
                   if(x == r_border[*hightest+1])
                   {
                       blake_line++;
                   }
               }
           }
       }
        if(blake_line >= 3)
        {
            if(l_loss_judge(l_border,90 ,110) == 0 && l_loss_judge(l_border,70 ,90) == 0
                    && r_loss_judge(r_border,90 ,110) == 0 && r_loss_judge(r_border,70 ,90) == 0
                    && monotonicity_l == 1 && monotonicity_r == 1)
                if(BridgeState == SINGLE_BRIDGE_NOT_ACTIVE && sum_island == 0 && island == 0
                        && cross_sum == 0)
                    jump_position = 1;
            if(eresting==1)
            {
                n=*hightest;
                m=(int)now_velocity;
                eresting=0;
            }
        }
    }
//    if(jump_position == 1)
//    {
//
//       if(*hightest < (uint8)jump_h)
//       {
//           jump_position = 0;
//       }
//    }
}

/*��
�������ƣ�void image_process(void)
����˵�������մ�����
����˵������
�������أ���
�޸�ʱ�䣺2025��1��5��
��    ע��
example�� image_process();
 */
void image_process(void)
{
uint8 hightest = 0;//����һ������У�tip����������ָ����yֵ����С
/*�������ߵ����õ�*/
Get_image(mt9v03x_image);//��ȡͼ��
turn_to_bin();//��ֵ��
/*��ȡ�����߽�*/
image_filter(bin_image);//�˲�
image_draw_rectan(bin_image);//Ԥ����
//����
data_stastics_l = 0;
data_stastics_r = 0;
data_stastics_l_fill = 0;
data_stastics_r_fill = 0;
if (get_start_point(image_h - 2))//�ҵ�����ˣ���ִ�а�����
{
    search_l_r((uint16)USE_num, bin_image, &data_stastics_l, &data_stastics_r, start_point_l[0], start_point_l[1], start_point_r[0], start_point_r[1], &hightest);

    // ����ȡ�ı߽�������ȡ���� �� ��������������õı���
    get_left(data_stastics_l);
    get_right(data_stastics_r);
    //������
    monotonicity_line(&hightest,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r,l_index,r_index);
    //ʮ�ֲ��ߺ���
    //cross_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r);
    //��Ծ
    jump_judge(bin_image,&hightest,l_border,r_border,monotonicity_l, monotonicity_r,(Encoder_Left-Encoder_Right)/2);
    //������
    cross_stop(bin_image,l_border,r_border);
    //����
    around_fill( bin_image,l_border, r_border, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index,monotonicity_l, monotonicity_r);
   //�������������Ҫ�ŵ�if����ȥ�ˣ���Ҫ�ŵ�if����ȥ�ˣ���Ҫ�ŵ�if����ȥ�ˣ���Ҫ����˵����

}
if(get_start_point(image_h - 2) == 0 && jump_position == 0) //����Ծû�ҵ���㣬����ͣ������
{
    target_velocity = 0;
    pit_all_close ();
    small_driver_set_duty(0, 0);
    string_not = 1;
}
//�������
//���������ж�

//���������ж�
for (int i = hightest; i < image_h-1; i++)
    {
    //���������ж�
    x1_boundary[i] = l_border[i];
    x2_boundary[i] = center_line[i];
    x3_boundary[i] = r_border[i];
    //���������ж�
    center_line[i] = (l_border[i] + r_border[i]) >> 1;//������
    //�������������󣬲����ǲ��߻�����״̬����ȫ�����ʹ��һ����ߣ����������������ܸ����������
    //��ȻҲ�ж�����ߵ��ҷ������Ǹ��˸о��ܷ�����������
    //���������ж�

    //���������ж�
    }
//���������ж�
//�����Ų���
bridge_fill(bin_image,l_border, r_border, center_line, data_stastics_l, data_stastics_r, dir_l, dir_r, points_l, points_r, &hightest, l_index, r_index);
//���������ж�

//��������Ӧ
if(jump_position == 0 && stop_position == 0 && BridgeState == SINGLE_BRIDGE_NOT_ACTIVE
       && cross_sum == 0 && island == 0 && sum_island == 0)
{
    buzzer = 0;
}
else
    buzzer = 1;
//����ȡ��
border = (float)((center_line[image_h-2] * 0.07) + (center_line[image_h-12] * 0.10)
       + (center_line[image_h-22] * 0.15) + (center_line[image_h-27] * 0.25)
       + (center_line[image_h-32] * 0.17) + (center_line[image_h-42] * 0.09)
       + (center_line[image_h-52] * 0.07) + (center_line[image_h-62] * 0.06)
       + (center_line[image_h-72] * 0.04));
//��������
s=stop_position;
u=cross_sum;
c=monotonicity_l;
e=monotonicity_r;
v=island;
f=sum_island;
q=BridgeState;
j=jump_position;
//��ʾͼ��
//tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);//��ʾԭͼ��
//tft180_show_gray_image (0,60, bin_image[0], image_w, image_h, image_w/2, image_h/2,0);//��ʾ��ֵͼ
//    for (int i = 0; i+1 < data_stastics_l; i++)
//    {
//        //tft180_draw_point((points_l[i][0]+2)/2, points_l[i][1]/2, RGB565_BLUE);//��ʾ����
//        tft180_draw_line((points_l[i][0]+2)/2, points_l[i][1]/2, (points_l[i+1][0]+2)/2, points_l[i+1][1]/2,RGB565_BLUE);
//    }
//    for (int i = 0; i+1 < data_stastics_r; i++)
//    {
//        //tft180_draw_point((points_r[i][0]-2)/2, points_r[i][1]/2, RGB565_RED);//��ʾ����
//        tft180_draw_line((points_r[i][0]-2)/2, points_r[i][1]/2, (points_r[i+1][0]-2)/2, points_r[i+1][1]/2,RGB565_RED);
//    }
//    for (int i = 0; i < image_h-1; i++)
//        {
//            tft180_draw_point(center_line[i]/2, i/2, RGB565_GREEN);// ��ʾ����
//            tft180_draw_point(l_border[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�����
//            tft180_draw_point(r_border[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�ұ���
//        }
}
