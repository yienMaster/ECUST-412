#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>
#include <execinfo.h>
#include <stdio.h>
#include <sys/time.h> // 添加头文件，支持itimerval和setitimer
#include <pthread.h>
#include <cmath>

// 自定义头文件
#include "gpio.h"
#include "pwm.h"
#include "servo.h"
#include "tools.h"
#include "control.h"

#include "motor.h"
#include "encoder.h"

#include "debug.h"

#define MOTOR_CHIP_R 0 // chip0pwm1右边电机
#define MOTOR_CHANNEL_R 2
#define MOTOR_CHIP_L 0 // chip0pwm2左边电机
#define MOTOR_CHANNEL_L 1
// #define SERVO_CHIP  6    //chip6pwm0舵机
// #define SERVO_CHANNEL 0
#define ENCODER_R 1            // chip7pwm0右边编码器//
#define ENCODER_DIRECTION_R 72 //
#define ENCODER_L 3            // chip5pwm0左边编码器//
#define ENCODER_DIRECTION_L 73 //
// 61 62作为电机方向控制引脚
#define MOTOR_DIRECTION_R 62 // 62变为按钮2
#define MOTOR_DIRECTION_L 61 // 61变为按钮1

using namespace cv;
using namespace std;



//extern bool is_debug ; // 调试开关转移至control.h:30

double target_speed = 32; // 目标速度  无效-> 详见 main.cpp:440行;

double encoder_value_R = 0;
double encoder_value_L = 0;
double target_speed_R = 0;
double target_speed_L = 0;
double control_speed_L = 0; // pid计算出来的速度
double control_speed_R = 0;
int control_direction_L = 1; // 控制方向
int control_direction_R = 1;
float pre_feed; // 声明前馈项

// 全局变量状态机
int flag_zebra = 0;    // 切换斑马线标志位计数
int state = st_normal; // 初始状态下正常直行
int before_state = st_normal;
int last_distance2 = 0;
int last_distance = 0;

Imfor info;

ENCODER encoder(ENCODER_R, ENCODER_DIRECTION_R);
ENCODER encoderL(ENCODER_L, ENCODER_DIRECTION_L);

Motor *motor = NULL;
Motor *motorL = NULL;
float diff_speed = 0.0;            // 动态差速值
const float DIFF_THRESHOLD = 55.0; // 差速激活阈值
const float DIFF_KP = 0.0045*2;      // 0.75;        // 0.33;   // 差速比例系数
const float diff_limit = 16+12+10+5;
//    35.0  0.3  diff_speed <=5

int sock = -1; // TCP 套接字

int sockfd_mid, sockfd_canny; // UDP 套接字

// 60  0.4

pthread_t motor_thread; // 电机线程的ID

pthread_t beep_thread; // 蜂鸣器线程句柄

PWM *pwm1 = NULL; // 全局Servo
PWM *pwm_fan=NULL;//负压全局fan

const bool need_fan=true;      //负压开关


VideoWriter writer, writer_canny, writer_bird;

VideoCapture cam(0);
// VideoCapture cam("./test_avi/ori.avi"); // 读取视频素材作为摄像头调试数据
//   需要取消反转 flip(,,-1)

PID pid_R = {0}; // 右轮PI控制器实例
PID pid_L = {0}; // 左轮PI控制器实例

pthread_mutex_t encoder_mutex = PTHREAD_MUTEX_INITIALIZER;

GPIO beep(63); // 蜂鸣器 GPIO 63

int width_table[120]={0};       //宽度表


bool is_right_circle;          //是否是右圆环

void *motor_update(void *arg) // 线程函数
{
    const int de_speed_limit=55;        //减速力度上限 0719 设为55
    while (1)
    {
        pthread_mutex_lock(&encoder_mutex);
        encoder_value_R = -encoder.pulse_counter_update();
        encoder_value_L = encoderL.pulse_counter_update();

        if (diff_speed > diff_limit)
            diff_speed = diff_limit; // 限制超调
        if (diff_speed < -1.0 * diff_limit)
            diff_speed = -1.0 * diff_limit;

        target_speed_R = target_speed + diff_speed / 2;
        target_speed_L = target_speed - diff_speed / 2;
        pthread_mutex_unlock(&encoder_mutex);
        control_speed_R = pid_calculate(&pid_R, encoder_value_R, target_speed_R); // 控制大小（力大小）
        control_speed_L = pid_calculate(&pid_L, encoder_value_L, target_speed_L);

        // 限制除圆环外的减速
        if (control_speed_L < -de_speed_limit && state != st_Rcircle_1 && state != st_Rcircle_2 && state != st_Rcircle_3 && state != st_Rcircle_4)
            control_speed_L = -de_speed_limit;
        if (control_speed_R < -de_speed_limit && state != st_Rcircle_1 && state != st_Rcircle_2 && state != st_Rcircle_3 && state != st_Rcircle_4)
            control_speed_R = -de_speed_limit;

        control_direction_R = control_speed_R >= 0 ? 0 : 1;
        control_direction_L = control_speed_L >= 0 ? 1 : 0;

        motor_set_direction(motor, control_direction_R);
        motor_set_direction(motorL, control_direction_L);
        motor_set_speed(motor, fabs(control_speed_R));  // 速度和方向control_speed_R
        motor_set_speed(motorL, fabs(control_speed_L)); // control_speed_L
        printf("右轮速度: %6.1f|目标: %6.1f||控制: %6.1f\n左轮速度: %6.1f|目标: %6.1f|控制: %6.1f|zebra_flag:%d\n",
               encoder_value_R, target_speed_R,control_speed_R,
               encoder_value_L, target_speed_L,control_speed_L, flag_zebra);

        // IMU_Data data = imu.calculate_angles();
        // printf("俯仰角: %.2f°  横滚角: %.2f°  偏航角: %.2f°\n",
        //      data.pitch, data.roll, data.yaw);

        usleep(5*1000); // 10ms更新一次电机PID  @25/07/18
    }
}

void *beep_s(void *arg) // 蜂鸣器线程函数0.3s鸣叫
{
    const int miliseconds = 300;            //0.3s
    beep.setValue(true);
    usleep(miliseconds * 1000);
    beep.setValue(false);
    return NULL;
}

void signal_handler(int sig)
{
    target_speed = 0;               // 速度归0
    pwm_set_duty_cycle(pwm1, 0.45); // 直着停止
    if(need_fan)    pwm_set_duty_cycle(pwm_fan,0);
    pthread_cancel(motor_thread);   // 终止线程

    beep.setValue(false);
    usleep(25000);

    pwm_destroy(pwm1);
    if(need_fan) pwm_destroy(pwm_fan);

    writer.release();
    writer_canny.release();
    writer_bird.release();

    motor_destroy(motor);
    motor_destroy(motorL);

    printf("硬件释放完成！\n");

    cout << "实际分辨率: " << cam.get(CAP_PROP_FRAME_WIDTH) << "x" 
     << cam.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "实际帧率: " << cam.get(CAP_PROP_FPS) << endl; // 日志显示仅30fps
    void *array[50];
    size_t size = backtrace(array, 50);
    backtrace_symbols_fd(array, size, STDERR_FILENO);  // 直接输出到stderr

    if (sock != -1)
        close(sock); // 关闭套接字

    close(sockfd_mid);
    close(sockfd_canny);
    cout << sig << " handled successfully." << endl;
    _exit(0); // exit
}

int main(void)
{ // atexit(cleanup);
    // std::set_terminate(terminateHandler);

    //  //启动步骤   按钮、LED1 和 LED2
    GPIO button(75);
    GPIO led1(48);
    GPIO led2(49);
    beep.setDirection("out");
    beep.setValue(false);

    // 设置按钮为输入模式
    button.setDirection("in");
    // 设置 LED1 和 LED2 为输出模式
    led1.setDirection("out");
    led2.setDirection("out");

    // 初始时 LED1 亮
    led1.setValue(true);
    

    bool buttonPressed = false;
    // 循环检测按钮是否按下
    //  while (!buttonPressed)
    {
        std::cout << "检测按钮中" << std::endl;
        buttonPressed = !button.readValue();
        usleep(100000); // 延时 100ms
    }

    // 按钮按下后，LED1 灭，LED2 亮
    led1.setValue(false);
    led2.setValue(false);

    std::cout << "按钮已按下，程序启动" << std::endl;

    motor = motor_init(MOTOR_CHIP_R, MOTOR_CHANNEL_R);
    motorL = motor_init(MOTOR_CHIP_L, MOTOR_CHANNEL_L);
    pwm1 = pwm_init(6, 0); // 全局Servo
    std::cout << "电机，舵机结构体初始化完成" << std::endl;

    auto end_zebra = std::chrono::high_resolution_clock::now();
   // cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_DEBUG); // 启用opencv 调试信息

    give_width();           //宽度表初始化

    signal(SIGINT, signal_handler);                                       // 注册Ctrl+C信号的处理函数
    signal(SIGSEGV, signal_handler);                                      // 注册err 信号处理器
    signal(SIGABRT, signal_handler);                                      // 注册abort 信号处理器
    signal(SIGBUS, signal_handler);                                       // 注册Bus信号处理器

    pid_init(&pid_R, 3.0, 0.12, 0, 0.8, target_speed); // 参数分别为//kp,ki,kd kf  target 3.3*0.8  pid_init(&pid_R, 3.0 ,0.02 ,0 ,0.7, target_speed);
    pid_init(&pid_L, 3.0, 0.12, 0, 0.8, target_speed); // 4.2*0.8
    std::cout << "PID initialized." << std::endl;

    motor->direction_channel = MOTOR_DIRECTION_R; // 控制方向初始化
    motorL->direction_channel = MOTOR_DIRECTION_L;
    motor_init_direction(motor);
    motor_init_direction(motorL);
    std::cout << "Motor initialized." << std::endl;

    set_cam(cam); // 设置摄像头参数

    // 初始化互斥锁
    if (pthread_mutex_init(&encoder_mutex, NULL) != 0)
    {
        cout << "互斥锁初始化失败!" << endl;
        exit(EXIT_FAILURE);
    }
    if (pthread_create(&motor_thread, NULL, motor_update, NULL) != 0) // 创建电机线程
    {
        cout << "创建线程失败!" << endl;
        raise(SIGINT);
    }

    PIDController steering_pid;

    // int state = st_normal; // 初始状态下正常直行
    float motor_pwm = 0.45;

    // jzy servo
    pwm_set_frequency(pwm1, 300);
    pwm_set_duty_cycle(pwm1, motor_pwm);
    pwm_enable(pwm1);

    //风扇
    if(need_fan)
    { pwm_fan=pwm_init(4,0);
    pwm_set_frequency(pwm_fan,50);
    pwm_set_duty_cycle(pwm_fan,0.085);      //风扇6%-10%
    pwm_enable(pwm_fan);
    }

    // tcp_debug("192.168.137.1", 1347);

    struct sockaddr_in remote_addr_mid = udp_connect(&sockfd_mid, 8888);     // mid 视频
    struct sockaddr_in remote_addr_canny = udp_connect(&sockfd_canny, 8889); // canny

    Mat img;

    vector<Point> judge_points = {}; // 前瞻点
    judge_points.reserve(5);         // 提前分配5个点的大小  避免频繁扩容带来的性能下降
    for (int i = DOWN_DETECT; i >= UP_DETECT; i--)
    {
        judge_points.push_back(Point(CAR_X, i));
    }

    int corner = 0;

    const int sm_border_th = 55; // 左下/右下固定角点距离图像中点的距离   应与cross 函数中保持一致(一般情况下)
    const Point left_down_corner = Point(CAR_X - sm_border_th, IMG_HEIGHT - 1);
    const Point right_down_corner = Point(CAR_X + sm_border_th, IMG_HEIGHT - 1);

    while (1)
    {
        cam >> img;      // 获取一帧图像
        if (img.empty()) // 视频结束
        {
            cout << "empty frame." << endl;
            raise(SIGINT);
        }
        auto start = std::chrono::high_resolution_clock::now(); // 开始计时

        Mat gray, edge; // 灰度图像和边缘检测图像
        Mat mid;        // 用于绘制中线的图像
        Mat assess;     // 评估丢线的图像
        Mat bin;        // 二值化图像

        int gap = 0;

        float a1 = 0, b1 = 0, a2 = 0, b2 = 0;

        // int left_lost = 0, right_lost = 0; // 丢线情况

        vector<Point> left_border = {}, right_border = {}, mid_border = {};           // 赛道左右边界点 和中线 坐标
        left_border.reserve(140), right_border.reserve(140), mid_border.reserve(140); // 预分配内存 避免频繁扩容

        vector<Point> max_white_col = {}; // 最长白列的点集
        max_white_col.reserve(120);       // 预分配内存

        vector<Point> right_angles = {}, acute_angles = {}; // 存储直角和锐角的点
        right_angles.reserve(10), acute_angles.reserve(10);

        vector<Point> left_corners = {}, right_corners = {};
        left_corners.reserve(10), right_corners.reserve(10); // 角点

        vector<Point> corners = {};
        corners.reserve(5); // 预分配空间

        struct cross_lost lost_point;
        lost_point.left.reserve(50), lost_point.right.reserve(50);

        Point left_down_corner = Point(-1, -1), right_down_corner = Point(-1, -1), left_up_corner = Point(-1, -1), right_up_corner = Point(-1, -1); // 角点

        Point single_up_corner = Point(-1, -1), single_down_corner = Point(-1, -1); // 单侧角点

        vector<Point> left_valid_border = {}, right_valid_border = {}; // 存储左有效边界点和右有效边界点
        left_valid_border.reserve(120), right_valid_border.reserve(120);

        bool use_down_corner = false; // 是否使用了下方固定角点

        Vec4f left_assess_line, right_assess_line; // 赛道左右丢线评估线

        flip(img, img, -1); // 垂直+水平翻转图像

        cvtColor(img, gray, COLOR_BGR2GRAY); // 灰度化

        GaussianBlur(gray, gray, Size(5, 5), 0); // 高斯模糊
        // 仅处理灰度化的图像还是会快很多
        mid = gray.clone();

        /*   逆透视变换
        // 顺序必须是 左上 -> 右上 -> 右下 -> 左下
        vector<Point2f> srcPoints = {{1, 30}, {159, 30}, {160, 120}, {0, 120}};
        vector<Point2f> dstPoints = {{1, 30}, {159, 30}, {100, 120}, {60, 120}};

        // 计算变换矩阵
        Mat M = getPerspectiveTransform(srcPoints, dstPoints);

        // 应用逆透视变换
        Mat img_birdview;
        warpPerspective(
            gray,
            img_birdview,
            M,
            Size(160, 120),
            INTER_LINEAR,
            BORDER_CONSTANT,
            Scalar(0) // 填充黑色边界（可选）
        );
        */

        threshold(gray, bin, 0, 255, THRESH_BINARY | THRESH_OTSU); // 大津法 二值化
        Canny(gray, edge, 50, 150);                                // 边缘检测

        // ========== 修改为使用Canny图像的斑马线检测 ==========
        const int ZEBRA_ROW = DOWN_DETECT + 15;
        const int CHECK_WIDTH = 50;
        int transition_count = 0;
        bool last_state = edge.at<uchar>(ZEBRA_ROW, CAR_X) > 0; // 改为检测边缘像素

        // 横向扫描检测行
        for (int x = CAR_X - CHECK_WIDTH; x <= CAR_X + CHECK_WIDTH; x++)
        {
            if (x < 0 || x >= edge.cols)
                continue;

            bool current_state = edge.at<uchar>(ZEBRA_ROW, x) > 0; // 使用Canny图像
            if (current_state != last_state)
            {
                transition_count++;
                last_state = current_state;
            }
        }

        // 调整有效性条件（边缘更密集需要更高阈值）
        bool valid_zebra = (transition_count >= 12); //&& // 提高阈值到8个边缘跳变
                                                     //(transition_count % 2 == 0) &&
                                                     //(state == st_normal||st_Rcircle_1);

        // 更新斑马线状态（需要更严格的连续检测）
        static int zebra_counter = 0;
        zebra_counter = valid_zebra ? zebra_counter + 1 : 0;
        if (zebra_counter >= 1) // 需要连续2帧检测到
        {
            state = st_zebra;
            led1.setValue(true);
            led2.setValue(true);
        }

        Imfor info = get_border_points_and_get_gap(edge, left_border, right_border, mid_border,
                                                   max_white_col, bin, lost_point,
                                                   left_valid_border, right_valid_border);

        // detectAngles(left_border, right_angles, mid, 3, 40);
        // detectAngles(right_border, right_angles, mid, 3, 40);

        std::cout << "Gap: " << info.gap << std::endl;
        std::cout << "远处Distance: " << info.distance << std::endl; //(y>=38&&y<=40)//高处
        std::cout << "近处前瞻点Distance: " << info.distance2 << std::endl;
        gap = info.gap;


        int la_state = state;
        if (state != st_zebra)
            {state = get_state(state, left_border, right_border, lost_point.left, lost_point.right, corners); // 获取状态
            //if(la_state==st_left_obstacle||la_state==st_right_obstacle)

    if(info.distance <=25&&info.distance>=22&&last_distance-info.distance>=0&&last_distance-info.distance<=4&&state==st_normal&&is_border_continous(right_border,2.5)&&abs(gap)<300) {//9.5 2.5  5
        //if(info.distance <20&&info.distance>14&&last_distance>=info.distance&&state==st_normal) {   
          state= st_left_obstacle;}
     if(info.distance <=25&&info.distance>=22&&last_distance-info.distance>=0&&last_distance-info.distance<=4&&state==st_normal&&is_border_continous(left_border,2.5)&&abs(gap)<300) {//9.5 2.5  5
        //if(info.distance <20&&info.distance>14&&last_distance>=info.distance&&state==st_normal) {   
          state= st_right_obstacle;}
     }

        if (la_state != state)
        {
            if (pthread_create(&beep_thread, NULL, beep_s, NULL) != 0) // 创建电机线程
            {
                cout << "创建线程失败!" << endl;
                raise(SIGINT);
            }
        }


        //----------------急弯中线修正--------start-------------//
        vector<Point> mid_c={};     //修正后的中线

        if(state==st_normal)
        {
            if(lost_point.left.size()>left_border.size()*0.6)          //左边界丢线超60%
            {
                const int c_gap=correct_mid(right_valid_border,true,mid_c,gap);
                if(state>=1&&state<=5)
                gap=c_gap;
                else
                gap=c_gap*1.4;      //根据右边界修正
            }   
            else if (lost_point.right.size()>right_border.size()*0.6)      //右边界丢线超60%
            {
                 const int c_gap=correct_mid(left_valid_border,false,mid_c,gap);    
                if(state>=1&&state<=5)
                gap=c_gap;
                else
                gap=c_gap*1.4;      //根据左边界修正
            }
        }

        //----------------急弯中线修正----------end-----------//

        if(state == st_Rcircle_1)
             target_speed=18;
        else  if (  state == st_Rcircle_2 || state ==st_Rcircle_3 ||  state ==st_Rcircle_4 )
            target_speed = 25;
        else if(gap>500||gap<-500||state==st_cross||(info.distance==0&&state==st_normal))
            target_speed = 29-3;
        else if(gap>350||gap<-350)
            target_speed = 34-3;
        else if(gap>200||gap<-200)
            target_speed=35-3;
        else if(state==st_left_obstacle||state==st_right_obstacle)
            target_speed=35-3;
       // else if()
        //    target_speed=36-3;
        else if(gap>120||gap<-120||state ==st_Rcircle_5)
            target_speed=38-3-1;
        else if(info.distance2>=60&&info.distance2<=70&&info.distance<=29&&info.distance>=31)
            target_speed=50-5;
        else
            target_speed=47-5-2;


        target_speed=25+5;    

    
        last_distance = info.distance;
        last_distance2 = info.distance2;
        switch (state)
        {               // 状态机 根据当前状态选择巡线策略
        case st_normal: // 正常状态
            led1.setValue(false);
            led2.setValue(false);
            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;

        case st_Rcircle_1: // 直行但准备入环
        {                  // 大括号限制局部变量 le 的作用域
            led1.setValue(false);
            led2.setValue(true);
            gap=0;
            //is_right_circle=false;
            if(is_right_circle)
            gap=get_single_gap(left_border,true,45);            //单侧左巡线
            else
            gap=get_single_gap(right_border,false,25);
            
            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        }
        case st_Rcircle_2: // 入环
        {
            led1.setValue(true);
            led2.setValue(false);
            gap = 0;
            if (is_right_circle) // 右圆环
                //gap=-300;
                gap=-550;
               //gap=get_single_gap(right_border,false,25);            //单侧右巡线
            else
                //gap=300;   
                gap=450;
                //gap=get_single_gap(left_border,true,28);            //单侧左巡线
            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        }

        case st_Rcircle_3: // 环内弯道  给一定偏移量
            if (is_right_circle)
                gap *= 1.1;
                //gap=get_single_gap(right_border,false,20);            //单侧右巡线
            else
                gap *=1.1;
               // gap=get_single_gap(left_border,true,20);            //单侧左巡线


            led1.setValue(false);
            led2.setValue(false);
            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;

        case st_Rcircle_4:
            led1.setValue(true);
            led2.setValue(true);

            if (is_right_circle)  //25-800    27-
                gap=-1250;           //-650  大概率出界(小圆环)   750 小概率
                //gap=get_single_gap(right_border,false,30);            //单侧右巡线
            else
                gap=1150;
                //gap=get_single_gap(left_border,true,30);            //单侧左巡线

            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;

        case st_Rcircle_5:          // 出环直行  与入环执行 st_Rcircle_1类似
            led1.setValue(false);
            led2.setValue(false);
      

                if (is_right_circle) // 右圆环
                    gap=get_single_gap(left_border,true,30);            //单侧左巡线
                else
                    gap=get_single_gap(right_border,false,30);            //单侧右巡线


            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);

            break;

        case st_cross: // 交叉路口
            if (lost_point.L_y_min > DOWN_DETECT && lost_point.R_y_min > DOWN_DETECT)
            {
                state = st_normal;
            } // 已通过
            // gap=cross_gap(left_valid_border,right_valid_border,left_lost_points,right_lost_points,gap,mid);
            if (state == st_cross)
            {
                cross_corner_detect(left_border, right_border, lost_point, left_down_corner,
                                    left_up_corner, right_down_corner, right_up_corner); // 获取交叉路口角点
                // x=ay+b
                if ((left_down_corner.x > 0 && left_down_corner.y > 0) && (left_up_corner.x > 0 && left_up_corner.y > 0) &&
                    (right_down_corner.x > 0 && right_down_corner.y > 0) && (right_up_corner.x > 0 && right_up_corner.y > 0))
                {

                    a1 = (float)(left_up_corner.x - left_down_corner.x) / (float)(left_up_corner.y - left_down_corner.y);
                    b1 = left_up_corner.x - a1 * left_up_corner.y;
                    // a1,b1->左

                    a2 = (float)(right_up_corner.x - right_down_corner.x) / (float)(right_up_corner.y - right_down_corner.y);
                    b2 = right_up_corner.x - a2 * right_up_corner.y;
                    // a2,b2->右
                    int cross_gap = 0;
                    for (int y = DOWN_DETECT; y >= UP_DETECT; y--)
                    {
                        int mid_x = (a1 * y + b1 + a2 * y + b2) / 2;
                        cross_gap += CAR_X - mid_x;
                    }
                    gap = cross_gap;
                }
            }

            motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;

        case st_zebra:           // 斑马线状态
            if (flag_zebra != 0) // 5s后解除锁定
            {
                target_speed = 0; // stop
                cout << "--------------zebra!!!------------" << endl;
                pwm_set_duty_cycle(pwm1, 0.45); // 直着停止
                sleep(2);
                raise(SIGINT);
            }
            else
            {
                state = st_normal; // 第一次过完斑马线后要恢复正常状态！
                cout << "--------------第一次通过斑马线------------" << endl;
                motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
                pwm_set_duty_cycle(pwm1, motor_pwm);
            }
            break;
        case st_left_obstacle: // 左边避障，右侧巡线
                               // target_speed = 15;
            if (info.distance2 >= 55 && last_distance2 >= 55)
            {
                cout << "--------------st_left_obstacle!!!------------" << endl;
                led1.setValue(true);
                led2.setValue(true);
                gap = (0.3*get_single_gap(right_border, false, 15) + 0.7 * gap);
                motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
                pwm_set_duty_cycle(pwm1, motor_pwm); // 模仿出环岛
            }
            else // 前瞻点进入障碍处
            {
                state = st_normal;
                motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
                pwm_set_duty_cycle(pwm1, motor_pwm);
                cout << "--------------出obstacle!!!------------" << endl;
            }
            break;
        case st_right_obstacle: // 左侧避障，右侧巡线
            //target_speed = 15;
            if (info.distance2 >= 55 && last_distance2 >= 55)
            {
                cout << "--------------st_right_obstacle!!!------------" << endl;
                led1.setValue(true);
                led2.setValue(true);
                gap = (0.3 * get_single_gap(left_border, true, 15) + 0.7 * gap);
                motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
                pwm_set_duty_cycle(pwm1, motor_pwm); // 模仿出环岛
            }
            else // 前瞻点进入障碍处
            {
                state = st_normal;
                motor_pwm = PID_Servo_WithDelayComp(steering_pid, (float)gap);
                pwm_set_duty_cycle(pwm1, motor_pwm);
                cout << "--------------出obstacle!!!------------" << endl;
            }
            break;

        default:
            cout << "------------------" << endl;
            cout << "-----意外状态------" << endl;
            cout << state << endl;
            raise(SIGSEGV);
            break;
        }

        // cout << "LE:" << left_lost << "\tRE:" << right_lost << "\tS:" << state << endl;

        // update_diff_speed((float)gap, target_speed); // 更新差速,

        // 差速
        if (abs(gap) > DIFF_THRESHOLD)
        {
            // 计算转向比：基于gap超出阈值的部分
            float steer_ratio = (gap - (gap > 0 ? DIFF_THRESHOLD : -DIFF_THRESHOLD)) / 100.0f;
            // 指数曲线控制：使用e^x函数，使小误差时响应平缓，大误差时响应迅速
            float exp_ratio = (exp(fabs(steer_ratio)) - 1) / (exp(1) - 1); // 归一化到[0,1]     //-1 -1 @27
            diff_speed = DIFF_KP * target_speed * exp_ratio * (steer_ratio > 0 ? 1 : -1);
        }
        else
        {
            diff_speed = 0.0; // 小误差时禁用差速
        }

        

        auto end1 = std::chrono::high_resolution_clock::now();
        // 处理完1帧图像
        std::chrono::duration<double, std::milli> elapsed = end1 - start;

        cout << elapsed.count() << "ms" << "\toutput:" << motor_pwm << "\tgap:" << gap << endl; // 处理1帧图像耗时
        if (is_debug)
        {
            if (sock != -1)
            {
                char msg[50] = "";
                sprintf(msg, "0,%d,%.2f,%.2f,%.2f\n", gap, pre_feed, encoder_value_L, encoder_value_R);
                send(sock, msg, strlen(msg), 0);
            }
            /*
            else
            {
                cout << "未连接TCP" << endl;
                raise(SIGINT);
            }
                */

            //--------!!!------
            cvtColor(mid, mid, COLOR_GRAY2BGR); // 将canny图像转为3通道 方便观看
            // malloc(): invalid next size (unsorted)
            //--------!!!------

            // detectAngles(left_border, left_corners, mid, 3);
            // detectAngles(right_border, right_corners, mid, 3);

            draw_by_points(left_border, mid, 0, 255, 0); // 绘制边界  绿色
            draw_by_points(right_border, mid, 0, 255, 0);
            draw_by_points(mid_border, mid, 255, 0, 0); // 绘制中线 红色

            draw_by_points(judge_points, mid, 0, 0, 255); // 前瞻点 蓝色

            draw_by_points(lost_point.left, mid, 128, 0, 128); //   丢线紫色

            draw_by_points(lost_point.right, mid, 128, 0, 128); // 紫色

            draw_by_points(mid_c,mid,255, 165, 0);      //绘制修正中线

            // cout<<"max_white_col.size():"<<max_white_col.size()<<endl;
            draw_by_points(max_white_col, mid, 0, 255, 255); // 最长白列用青色标出

            //drawline(mid, left_assess_line, 255, 165, 0); // 绘制橙色评估线
            //drawline(mid, right_assess_line, 255, 165, 0);

            // 显示图像
            putText(mid, "LE:" + to_string(lost_point.left.size()), Point(5, 15), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);  // 左线误差
            putText(mid, "RE:" + to_string(lost_point.right.size()), Point(5, 25), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA); // 右线误差

            putText(mid, "g:" + to_string(gap), Point(80, 15), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "p:" + to_string_p((double)motor_pwm, 2), Point(80, 25), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "S:" + to_string(state), Point(5, 35), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "R" + to_string_p(encoder_value_R, 2), Point(80, 95), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "" + to_string_p(target_speed_R, 2), Point(80, 105), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "L" + to_string_p(encoder_value_L, 2), Point(5, 95), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "" + to_string_p(target_speed_L, 2), Point(5, 105), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);

            putText(mid, ".", Point(5, 43), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, ".", Point(5, 45), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
            putText(mid, "ds:" + to_string_p(info.distance, 2), Point(5, 85), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);

            if (state == st_cross)
            {
                putText(mid, "D", Point(5, 45), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
                circle(mid, left_down_corner, 2.5, Scalar(128, 0, 128), -1);
                circle(mid, left_up_corner, 2.5, Scalar(128, 0, 128), -1);
                circle(mid, right_down_corner, 2.5, Scalar(128, 0, 128), -1);
                circle(mid, right_up_corner, 2.5, Scalar(128, 0, 128), -1);

                for (int y = right_down_corner.y - 1; y > right_up_corner.y; y--)
                {
                    const int x = a2 * y + b2;
                    if (x <= 119 && x >= 0)
                    {
                        mid.at<Vec3b>(y, x)[0] = 255; // B
                        mid.at<Vec3b>(y, x)[1] = 0;   // G
                        mid.at<Vec3b>(y, x)[2] = 0;   // R
                    }
                }
                for (int y = left_down_corner.y - 1; y > left_up_corner.y; y--)
                {
                    const int x = a1 * y + b1;
                    if (x <= 119 && x >= 0)
                    {
                        mid.at<Vec3b>(y, x)[0] = 255; // B
                        mid.at<Vec3b>(y, x)[1] = 0;   // G
                        mid.at<Vec3b>(y, x)[2] = 0;   // R
                    }
                }

                /*
                static int count=0;
                save_pic(mid,"./corners/"+to_string(count)+".jpg");
                count++;
                */
            }
            if (state == st_Rcircle_1 || st_Rcircle_2 || st_Rcircle_3) // 布线
            {
                Point tmp = right_down_corner;
                if (use_down_corner) // 使用了下方角点
                {
                    if (!is_right_circle) // 左圆环
                        tmp = left_down_corner;
                }
                else
                    tmp = corners[0];
                circle(mid, tmp, 2.5, Scalar(0, 128, 128), -1);
                for (int y = tmp.y - 1; y > UP_DETECT - 5; y--)
                {
                    const int x = a1 * y + b1;
                    if (x <= 119 && x >= 0)
                    {
                        mid.at<Vec3b>(y, x)[0] = 255; // B
                        mid.at<Vec3b>(y, x)[1] = 0;   // G
                        mid.at<Vec3b>(y, x)[2] = 200; // R
                    }
                }
            }

            /*
            for (int i = 0; i < left_corners.size(); i++)
                circle(mid, left_corners[i], 2.5, Scalar(128, 0, 128), -1);



            for(int i=0;i<right_corners.size();i++)
                circle(mid, right_corners[i], 2.5, Scalar(128, 0, 128), -1);
            */

            if (corners.size() > 0)
            {
                for (int i = 0; i < corners.size(); i++)
                    circle(mid, corners[i], 2.5, Scalar(128, 0, 128), -1);
                // save_pic(mid,"./corners/"+to_string(corner)+".jpg");
                // corner++;
            }

            /*
            if(single_down_corner.x > 0&&single_down_corner.y>0)
            {

            }
                */

            writer.write(mid); // 写入视频文件

            cvtColor(edge, edge, COLOR_GRAY2BGR); // 将canny图像转为3通道 方便观看
            // cvtColor(img_birdview, img_birdview, COLOR_GRAY2BGR);

            writer_canny.write(img);


            /*
            save_pic(edge, "edge.jpg");
            save_pic(img, "ori.jpg");
            save_pic(mid, "mid.jpg");
            raise(SIGINT);
            */

            // writer_bird.write(img_birdview);
        }
        auto end2 = std::chrono::high_resolution_clock::now();
        elapsed = end2 - end_zebra; // debug耗时
        cout << "斑马线隔离时间:" << elapsed.count() << "ms" << endl;
        if (elapsed.count() > 8000)
            flag_zebra = 1;

        // udp_send_frame(mid, sockfd_mid, remote_addr_mid, 90);
        // udp_send_frame(edge, sockfd_canny, remote_addr_canny, 90);      //UDP发送

        if (right_angles.size() > 0)
        {
            // save_pic(mid,to_string(corner)+".jpg");
            // corner++;
        }

        /*
        save_pic(img,"1_img.jpg");
        save_pic(mid,"1_mid.jpg");
        save_pic(edge,"1_edge.jpg");
    */

        auto end3 = std::chrono::high_resolution_clock::now();
        elapsed = end3 - end2; // debug耗时
        cout << "debug_send:" << elapsed.count() << "ms" << endl;
    }

    // sleep(50);
    // pwm_destroy(pwm1);
    // pwm_destroy(pwm2);

    raise(SIGINT);
    return 0;
}
