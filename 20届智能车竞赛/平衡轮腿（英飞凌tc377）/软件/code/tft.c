#include "tft.h"

int clean = 1;
int tft_show = 0;//��ʾ����
float now_start = 0;//��ʼ���б�־λ
float a = 650;//Ŀ���ٶȸ�ֵ
//float a = 600;
float V_a = 0;
float acc = 1;//��������
int y=35;//��λ��
int x=5;//��λ��
int tft_direction = 0;//��Ļ����
float pid_get[3]={0,0,0};
//�Ӽ���־λ
//=1��
//=0����
//=-1��
int add = 0;


void TFT_init(void)
{
    tft180_set_dir(tft_direction);
    tft180_set_color(RGB565_WHITE, RGB565_BLACK);
    tft180_init();
}

void KEY_init(void)
{
    gpio_init(KEY1, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY1 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY2, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY2 ���� Ĭ�ϸߵ�ƽ ��������
    gpio_init(KEY3, GPI, GPIO_HIGH, GPI_PULL_UP);           // ��ʼ�� KEY3 ���� Ĭ�ϸߵ�ƽ ��������
}

//��������
void TFT_KEYput(uint8 get_key1,uint8 get_key2,uint8 get_key3)
{
    //��ֵ�ٶ�
    V_a = a;
    //��ѹ����
    if((voltage_power_supply <= 10.4) && (voltage_power_supply >= 4) && (jump_position == 0))
    {
        tft_show = 14;
        tft180_clear();
        pit_all_close ();
        small_driver_set_duty(0, 0);
    }
    //��Ծ����
    if(jump_position == 1)
    {
        tft_show = 15;
        tft180_clear();
    }
    //��Ļ�˵�
    switch(tft_show)
    {
        case 0 ://�˵�
        {
            //�˵�������ʾ
            tft180_show_chinese(110, 0, 16, chinese_wu[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 17, 16, chinese_er[0], 1, RGB565_BLUE );
            tft180_show_chinese(5, 0, 16, chinese_caidan[0], 2, RGB565_WHITE);
            tft180_show_chinese(5, 18, 16, chinese_kaishiqianjin[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 36, 16, chinese_voltage[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 54, 16, chinese_camera[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 72, 16, chinese_setup[0], 3, RGB565_WHITE);
            tft180_show_chinese(5, 90, 16, chinese_turn[0], 6, RGB565_WHITE);
            tft180_show_chinese(5, 108, 16, chinese_jump[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_bridge[0], 6, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=35;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=34)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                tft_show = (y - 17)/18;
                tft180_clear();
                if(y==125)
                    y=125;
                else
                    y=143;
            }
            break;
        }
        case 1 ://��ʼǰ��
        {
            //�˵�������ʾ
            tft180_show_chinese(5, 0, 16, chinese_kaishiqianjin[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 18, 16, chinese_tv[0], 5, RGB565_WHITE);
            tft180_show_float (10, 36, a, 4, 2);//��ʾĿ���ٶ�
            tft180_show_chinese(5, 90, 16, chinese_start[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 108, 16, chinese_changetv[0], 7, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=107;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=106)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 107:
                    {
                        tft_show = 8;
                        tft180_clear();
                        system_delay_ms(1000);
                        now_start = a;         //����Ŀ���ٶ�
                        y=143;
                        stop_position = 0;
                        stop = 0;
                        break;//��ʼ����
                    }
                    case 125:
                    {
                        tft_show = 9;
                        tft180_clear();
                        y=143;
                        x=25;
                        break;//����Ŀ���ٶ�
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 2 ://��Դ����
        {
            //�˵�������ʾ
            tft180_show_chinese(5, 0, 16, chinese_voltage[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 18, 16, chinese_dianliang[0], 4, RGB565_WHITE);
            tft180_show_float(5, 36, voltage_power_supply, 2, 3);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key3)
            {
                if(y==143)
                {
                    tft_show = 0;
                    tft180_clear();
                    y=35;
                }
            }
            break;
        }
        case 3 ://ͼ����ʾ
        {
            y=tft180_height_max-2;
            //�˵�������ʾ
            tft180_show_chinese(5, tft180_height_max-19, 16, chinese_return[0], 5, RGB565_WHITE);
            tft180_show_int(100, 0, m, 2);
            tft180_show_int(100, 25, n, 2);
            tft180_show_int(95, 50, s, 1);
            tft180_show_int(110, 50, u, 1);
            tft180_show_int(95, 75, c, 1);
            tft180_show_int(110, 75, e, 1);
            tft180_show_int(95, 100, v, 1);
            tft180_show_int(110, 100, f, 1);
            tft180_show_int(95, 120, q, 1);
            tft180_show_int(110, 120, j, 1);
            tft180_show_float(0, 120, border, 3, 3);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            //��ʾͼ��
            tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);//��ʾԭͼ��
            tft180_show_gray_image (0,60, bin_image[0], image_w, image_h, image_w/2, image_h/2,0);//��ʾ��ֵͼ
                for (int i = 0; i+1 < data_stastics_l; i++)
                {
                    //tft180_draw_point((points_l[i][0]+2)/2, points_l[i][1]/2, RGB565_BLUE);//��ʾ����
                    tft180_draw_line((points_l[i][0]+2)/2, points_l[i][1]/2, (points_l[i+1][0]+2)/2, points_l[i+1][1]/2,RGB565_BLUE);
                }
                for (int i = 0; i+1 < data_stastics_r; i++)
                {
                    //tft180_draw_point((points_r[i][0]-2)/2, points_r[i][1]/2, RGB565_RED);//��ʾ����
                    tft180_draw_line((points_r[i][0]-2)/2, points_r[i][1]/2, (points_r[i+1][0]-2)/2, points_r[i+1][1]/2,RGB565_RED);
                }
                for (int i = 0; i < image_h-1; i++)
                    {
                        tft180_draw_point(x2_boundary[i]/2, i/2, RGB565_GREEN);// ��ʾ����
                        tft180_draw_point(x1_boundary[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�����
                        tft180_draw_point(x3_boundary[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�ұ���
                    }
            if(!get_key3)
            {
                if(y==tft180_height_max-2)
                {
                    tft_show = 0;
                    tft180_clear();
                    y=35;
                }
            }
            break;
        }
        case 4 ://����
        {
            tft180_show_chinese(5, 0, 16, chinese_setup[0], 3, RGB565_WHITE);
            tft180_show_chinese(5, 90, 16, chinese_tftchange[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 108, 16, chinese_wifi[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=107;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=106)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 107:
                    {
                        if(tft_direction == 0)
                            tft_direction = 1;
                        else
                            tft_direction = 0;
                        tft180_set_dir(tft_direction);
                        break;//��ת��Ļ
                    }
                    case 125:
                    {
                        break;//wifi����
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 5 ://ת��
        {
            pid_get[0] = motor_direction.kp;
            pid_get[1] = motor_direction.ki;
            pid_get[2] = motor_direction.kd;
            tft180_show_chinese(5, 0, 16, chinese_turn[0], 6, RGB565_WHITE);
            tft180_show_string(5, 18, "Kp:");
            tft180_show_float(60, 18, pid_get[0], 2, 3);
            tft180_show_chinese(5, 36, 16, chinese_chose[0], 4, RGB565_WHITE);
            tft180_show_string(5, 54, "Kp^2:");
            tft180_show_float(50, 54, pid_get[1], 1, 6);
            tft180_show_uint(115, 54, (int)(pid_get[1]*10000000)%10, 1);
            tft180_show_chinese(5, 72, 16, chinese_chose[0], 4, RGB565_WHITE);
            tft180_show_string(5, 90, "Kd:");
            tft180_show_float(60, 90, pid_get[2], 2, 3);
            tft180_show_chinese(5, 108, 16, chinese_chose[0], 4, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y!=125)
                    y+=36;
                else
                    y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=53;
                }
            }
            if(!get_key2)
            {
                if(y!=143)
                    y-=36;
                else
                    y-=18;
                tft180_clear();
                if(y<=52)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 53:
                    {
                        tft_show = 11;
                        tft180_clear();
                        y=143;
                        x=25;
                        acc = 0.001;
                        break;//kp����
                    }
                    case 89:
                    {
                        tft_show = 12;
                        tft180_clear();
                        y=143;
                        x=25;
                        acc = 0.0000001;
                        break;//kp^2����
                    }
                    case 125:
                    {
                        tft_show = 13;
                        tft180_clear();
                        y=143;
                        x=25;
                        acc = 0.001;
                        break;//kd����
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        y=35;
                        acc = 1;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 6 ://��Ծ
        {
            tft180_show_chinese(5, 0, 16, chinese_jump[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 18, 16, chinese_jump_h[0], 7, RGB565_WHITE);
            tft180_show_uint(40, 36, jump_h,3);//��ʾ��Ծλ��
            tft180_show_chinese(5, 108, 16, chinese_return[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 5, RGB565_WHITE);
            tft180_show_char(86, 67, '*');
            tft180_show_uint(95, 67, acc, 3);
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            //����
            if(y == 107)
             {
                 if(x==85)
                 {
                     tft180_show_char(16, 90, '(');
                     tft180_show_char(34, 90, ')');
                 }
                 else if(x==0)
                 {
                     tft180_show_char(76, 90, '(');
                     tft180_show_char(94, 90, ')');
                 }
             }
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y+=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y+=18;
                    tft180_clear();
                }
                if(y>=126)
                {
                    y=89;
                }
            }
            if(!get_key2)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y-=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y-=18;
                    tft180_clear();
                }
                if(y<=88)
                {
                    y=125;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 89:
                    {
                        acc*=10;
                        if(acc > 10)
                        {
                            acc=1;
                        }
                        break;
                    }
                    case 107:
                    {
                        jump_h = jump_h + add*(int)acc;
                        tft180_clear();
                        break;
                    }
                    case 125:
                    {
                        tft_show = 0;
                        tft180_clear();
                        acc=1;
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 7 ://������
        {
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=35;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=34)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                if(y==143)
                {
                    tft_show = 0;
                    tft180_clear();
                    y=35;
                }
            }
            break;
        }
        case 8 ://��ʼ����
        {
            if(stop_position == 1 && stop == 1)
            {
                tft_show = 10;
            }
            tft180_show_chinese(110, 0, 16, chinese_wu[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 17, 16, chinese_er[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 34, 16, chinese_zhan[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 51, 16, chinese_dui[0], 1, RGB565_BLUE );
            tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);//��ʾԭͼ��            for (int i = 0; i < image_h-1; i++)
            for (int i = 0; i < image_h-1; i++)
            {
              tft180_draw_point(x2_boundary[i]/2, i/2, RGB565_RED);// ��ʾ����
              tft180_draw_point(x1_boundary[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�����
              tft180_draw_point(x3_boundary[i]/2, i/2, RGB565_BLUE);//��ʾ��� ��ʾ�ұ���
            }
            tft180_show_chinese(5, 72, 16, chinese_rv[0], 5, RGB565_WHITE);
            tft180_show_int(81, 70,(int)((Encoder_Left-Encoder_Right)/2), 4);
            tft180_show_chinese(5, 90, 16, chinese_text[0], 3, RGB565_WHITE);
            tft180_show_int(45, 87, m, 1);
            tft180_show_chinese(67, 90, 16, chinese_text[0], 3, RGB565_WHITE);
            tft180_show_int(107, 87, n, 1);
            tft180_show_chinese(5, 108, 16, chinese_stop[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=125;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=124)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 125:
                    {
                        tft_show = 10;
                        tft180_clear();
                        target_velocity = 0;         //����Ŀ���ٶ�
                        y=143;
                        break;//ֹͣ����
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 9 ://����Ŀ���ٶ�
        {
            //�˵�������ʾ
            tft180_show_chinese(5, 0, 16, chinese_changetv[0], 7, RGB565_WHITE);
            tft180_show_chinese(5, 18, 16, chinese_tv[0], 5, RGB565_WHITE);
            tft180_show_float (10, 36, a, 4, 2);//��ʾĿ���ٶ�
            tft180_show_chinese(5, 72, 16, chinese_changedobble[0], 5, RGB565_WHITE);
            tft180_show_char(86, 67, '*');
            tft180_show_uint(95, 67, acc, 3);
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            tft180_show_chinese(5, 108, 16, chinese_start[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            if(y == 107)
             {
                 if(x==85)
                 {
                     tft180_show_char(16, 90, '(');
                     tft180_show_char(34, 90, ')');
                 }
                 else if(x==0)
                 {
                     tft180_show_char(76, 90, '(');
                     tft180_show_char(94, 90, ')');
                 }
             }
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y+=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y+=18;
                    tft180_clear();
                }
                if(y>=144)
                {
                    y=89;
                }
            }
            if(!get_key2)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y-=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y-=18;
                    tft180_clear();
                }
                if(y<=88)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 89:
                    {
                        acc*=10;
                        if(acc > 100)
                        {
                            acc=1;
                        }
                        break;
                    }
                    case 107:
                    {
                        a = a + (float)add*acc;
                        tft180_clear();
                        break;
                    }
                    case 125:
                    {
                        tft_show = 8;
                        tft180_clear();
                        system_delay_ms(1000);
                        now_start = a;         //����Ŀ���ٶ�
                        acc=1;
                        y=143;
                        stop_position = 0;
                        stop = 0;
                        break;//��ʼ����
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        acc=1;
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 10 ://ֹͣ����
        {
            tft180_show_chinese(110, 0, 16, chinese_wu[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 17, 16, chinese_er[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 34, 16, chinese_zhan[0], 1, RGB565_BLUE );
            tft180_show_chinese(110, 51, 16, chinese_dui[0], 1, RGB565_BLUE );
            tft180_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, MT9V03X_W, MT9V03X_H, image_w/2, image_h/2, 0);//��ʾԭͼ��            for (int i = 0; i < image_h-1; i++)
            for (int i = 0; i < image_h-1; i++)
            {
              tft180_draw_point(x2_boundary[i]/2, i/2, RGB565_RED);// ��ʾ����
              tft180_draw_point(x1_boundary[i]/2, i/2, RGB565_GREEN);//��ʾ��� ��ʾ�����
              tft180_draw_point(x3_boundary[i]/2, i/2, RGB565_BLUE);//��ʾ��� ��ʾ�ұ���
            }
            tft180_show_chinese(5, 72, 16, chinese_rv[0], 5, RGB565_WHITE);
            tft180_show_int(81, 70, (int)((Encoder_Left-Encoder_Right)/2), 4);
            tft180_show_chinese(5, 90, 16, chinese_text[0], 3, RGB565_WHITE);
            tft180_show_int(45, 87, m, 1);
            tft180_show_chinese(67, 90, 16, chinese_text[0], 3, RGB565_WHITE);
            tft180_show_int(107, 87, n, 1);
            tft180_show_chinese(5, 108, 16, chinese_start[0], 5, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                y+=18;
                tft180_clear();
                if(y>=144)
                {
                    y=125;
                }
            }
            if(!get_key2)
            {
                y-=18;
                tft180_clear();
                if(y<=124)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 125:
                    {
                        tft_show = 8;
                        tft180_clear();
                        system_delay_ms(1000);
                        now_start = a;         //����Ŀ���ٶ�
                        y=143;
                        stop_position = 0;
                        stop = 0;
                        break;//��ʼ����
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 11 ://����Kp
        {
            tft180_show_chinese(5, 0, 16, chinese_turn[0], 6, RGB565_WHITE);
            tft180_show_string(5, 18, "Kp:");
            tft180_show_float(5, 36, pid_get[0], 2, 3);
            tft180_show_chinese(5, 54, 16, chinese_changedobble[0], 5, RGB565_WHITE);
            tft180_show_char(5, 67, '*');
            tft180_show_float(14, 67, acc, 1,3);
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            tft180_show_chinese(5, 108, 16, chinese_returnlast[0], 6, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            if(y == 107)
             {
                 if(x==85)
                 {
                     tft180_show_char(16, 90, '(');
                     tft180_show_char(34, 90, ')');
                 }
                 else if(x==0)
                 {
                     tft180_show_char(76, 90, '(');
                     tft180_show_char(94, 90, ')');
                 }
             }
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y+=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y+=18;
                    tft180_clear();
                }
                if(y>=144)
                {
                    y=89;
                }
            }
            if(!get_key2)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y-=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y-=18;
                    tft180_clear();
                }
                if(y<=88)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 89:
                    {
                        acc*=10;
                        if(acc > 1)
                        {
                            acc=0.001;
                        }
                        break;
                    }
                    case 107:
                    {
                        pid_get[0] = pid_get[0] + (float)add*acc;
                        tft180_clear();
                        break;
                    }
                    case 125:
                    {
                        tft_show = 5;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=143;
                        break;//������һ��
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 12 ://����Kp^2
        {
            tft180_show_chinese(5, 0, 16, chinese_turn[0], 6, RGB565_WHITE);
            tft180_show_string(5, 18, "Kp^2:");
            tft180_show_float(5, 36, pid_get[1], 1, 6);
            tft180_show_uint(70, 36, (int)(pid_get[1]*10000000)%10, 1);
            tft180_show_chinese(5, 54, 16, chinese_changedobble[0], 5, RGB565_WHITE);
            tft180_show_string(5, 67, "*1*10^-");
            if((int)(acc*10000000) < 1000 && (int)(acc*10000000) >= 100 )
                tft180_show_uint(65, 67, 5, 3);
            if((int)(acc*10000000) < 100 && (int)(acc*10000000) >= 10 )
                tft180_show_uint(65, 67, 6, 3);
            if((int)(acc*10000000) < 10 )
                tft180_show_uint(65, 67, 7, 3);
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            tft180_show_chinese(5, 108, 16, chinese_returnlast[0], 6, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            if(y == 107)
             {
                 if(x==85)
                 {
                     tft180_show_char(16, 90, '(');
                     tft180_show_char(34, 90, ')');
                 }
                 else if(x==0)
                 {
                     tft180_show_char(76, 90, '(');
                     tft180_show_char(94, 90, ')');
                 }
             }
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y+=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y+=18;
                    tft180_clear();
                }
                if(y>=144)
                {
                    y=89;
                }
            }
            if(!get_key2)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y-=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y-=18;
                    tft180_clear();
                }
                if(y<=88)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 89:
                    {
                        acc*=10;
                        if(acc > 0.00001)
                        {
                            acc=0.0000001;
                        }
                        break;
                    }
                    case 107:
                    {
                        pid_get[1] = pid_get[1] + (float)add*acc;
                        tft180_clear();
                        break;
                    }
                    case 125:
                    {
                        tft_show = 5;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=143;
                        break;//������һ��
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 13 ://����Kd
        {
            tft180_show_chinese(5, 0, 16, chinese_turn[0], 6, RGB565_WHITE);
            tft180_show_string(5, 18, "Kd:");
            tft180_show_float(5, 36, pid_get[2], 2, 3);
            tft180_show_chinese(5, 54, 16, chinese_changedobble[0], 5, RGB565_WHITE);
            tft180_show_char(5, 67, '*');
            tft180_show_float(14, 67, acc, 1,3);
            tft180_show_char(25, 90, '+');
            tft180_show_char(85, 90, '-');
            tft180_show_chinese(5, 108, 16, chinese_returnlast[0], 6, RGB565_WHITE);
            tft180_show_chinese(5, 126, 16, chinese_return[0], 5, RGB565_WHITE);
            //����
            if(y == 107)
             {
                 if(x==85)
                 {
                     tft180_show_char(16, 90, '(');
                     tft180_show_char(34, 90, ')');
                 }
                 else if(x==0)
                 {
                     tft180_show_char(76, 90, '(');
                     tft180_show_char(94, 90, ')');
                 }
             }
            tft180_draw_line(5, (uint16)y, tft180_width_max-1, (uint16)y,RGB565_RED);
            if(!get_key1)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y+=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y+=18;
                    tft180_clear();
                }
                if(y>=144)
                {
                    y=89;
                }
            }
            if(!get_key2)
            {
                if(y == 107)
                {
                    if(x==25)
                    {
                        tft180_clear();
                        add = 1;
                        x=85;
                    }
                    else if(x==85)
                    {
                        tft180_clear();
                        x=0;
                        add = -1;
                    }
                    else
                    {
                        y-=18;
                        tft180_clear();
                        x=25;
                        add = 0;
                    }
                }
                else
                {
                    y-=18;
                    tft180_clear();
                }
                if(y<=88)
                {
                    y=143;
                }
            }
            if(!get_key3)
            {
                switch(y)
                {
                    case 89:
                    {
                        acc*=10;
                        if(acc > 10)
                        {
                            acc=0.001;
                        }
                        break;
                    }
                    case 107:
                    {
                        pid_get[2] = pid_get[2] + add*acc;
                        tft180_clear();
                        break;
                    }
                    case 125:
                    {
                        tft_show = 5;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=143;
                        break;//������һ��
                    }
                    case 143:
                    {
                        tft_show = 0;
                        tft180_clear();
                        PidChange(&motor_direction, pid_get[0], pid_get[1],pid_get[2]);
                        acc=1;
                        y=35;
                        break;//���ز˵�
                    }
                }
            }
            break;
        }
        case 14 ://��Դ����
        {
            tft180_show_chinese(5, 0, 16, chinese_vlow[0], 3, RGB565_RED);
            tft180_show_chinese(5, 18, 16, chinese_pleaselow[0], 4, RGB565_RED);
            tft180_show_float(5, 36, voltage_power_supply, 2, 3);
            break;
        }
        case 15 ://��Ծ����
        {
            tft180_show_int(20, 0, m, 3);
            tft180_show_int(20, 25, n, 3);
            break;
        }
    }
}

