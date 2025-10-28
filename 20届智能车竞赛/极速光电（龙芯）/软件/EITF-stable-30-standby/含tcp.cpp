#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>
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
#define MOTOR_CHANNEL_R 1
#define MOTOR_CHIP_L 0 // chip0pwm2左边电机
#define MOTOR_CHANNEL_L 2
// #define SERVO_CHIP  6    //chip6pwm0舵机
// #define SERVO_CHANNEL 0
#define ENCODER_R 3 // chip7pwm0右边编码器//
#define ENCODER_DIRECTION_R 72
#define ENCODER_L 1 // chip5pwm0左边编码器//
#define ENCODER_DIRECTION_L 73

using namespace cv;
using namespace std;

double encoder_value_R = 0;
double encoder_value_L = 0;
double target_speed_R = 0;
double target_speed_L = 0;
double control_speed_L = 0; // pid计算出来的速度
double control_speed_R = 0;
float target_speed = 18;

ENCODER encoder(ENCODER_R, ENCODER_DIRECTION_R);
ENCODER encoderL(ENCODER_L, ENCODER_DIRECTION_L);

Motor *motor = motor_init(MOTOR_CHIP_R, MOTOR_CHANNEL_R);
Motor *motorL = motor_init(MOTOR_CHIP_L, MOTOR_CHANNEL_L);

float diff_speed = 0.0;            // 动态差速值
const float DIFF_THRESHOLD = 25.0; // 差速激活阈值
const float DIFF_KP = 0.33;        // 0.33;   // 差速比例系数
//    35.0  0.3  diff_speed <=5

int sock = 0;       //TCP 套接字

// 60  0.4

pthread_t motor_thread; // 电机线程的ID

PWM *pwm1 = pwm_init(6, 0); // 全局Servo
VideoWriter writer;

 VideoCapture cam(0);
//VideoCapture cam("./test_avi/test1.avi"); // 读取视频素材作为摄像头调试数据
// 需要取消反转 flip(,,-1)

// PID 控制结构体
struct PID
{
    double target;          // 目标值
    double kp;              // 比例系数
    double ki;              // 积分系数
    double integral;        // 积分项
    double last_error;      // 上次误差
    double kf;              // 前馈系数
    double filter_alpha;    // 新增输出滤波系数 [0-1]
    double filtered_output; // 新增滤波状态
};

PID pid_R = {0}; // 右轮PI控制器实例
PID pid_L = {0}; // 左轮PI控制器实例

// 初始化
void pid_init(PID *pid, double kp, double ki, double kf, double fi, double target)
{
    pid->kp = kp;
    pid->ki = ki;

    pid->kf = kf;             // 前馈系数
    pid->filter_alpha = fi;   // 输出滤波系数
    pid->filtered_output = 0; // 初始化滤波状态

    pid->target = target;
    pid->integral = 0;
    pid->last_error = 0;
}

// PI控制计算函数，执行顺序：死区处理→滤波→限幅
double pid_calculate(PID *pid, double feedback, double target)
{
    double error = target - feedback;
    // 前馈项（新增）
    double feedforward = pid->kf * target;
    // 比例项
    double p_out = pid->kp * error;

    // 积分项（带抗饱和）
    const double integral_max = 25.0;
    pid->integral += error;
    if (pid->integral > integral_max)
        pid->integral = integral_max;
    else if (pid->integral < -integral_max)
        pid->integral = -integral_max;

    double i_out = pid->ki * pid->integral;

    // 保存本次误差
    pid->last_error = error;

    // 计算原始输出
    double output = feedforward + p_out + i_out;

    // 死区控制
    if (fabs(error) < 0.15)
    {
        output = feedforward;
        if (fabs(error) < 0.03)
        {
            pid->integral *= 0.96;
        }
        else
        {
            // 动态衰减系数调整
            // 动态衰减系数（0.8~0.95）
            double alpha = 0.8 + (0.15 - fabs(error)) / 0.15 * 0.15;
            pid->integral *= alpha;
        }
    }

    // 应用低通滤波
    pid->filtered_output = pid->filter_alpha * pid->filtered_output + (1 - pid->filter_alpha) * output;

    // 输出限幅
    const double output_max = 100;
    const double output_min = 0;
    cout << "return:" << fmax(fmin(pid->filtered_output, output_max), output_min) << endl;
    return fmax(fmin(pid->filtered_output, output_max), output_min);
}

pthread_mutex_t encoder_mutex = PTHREAD_MUTEX_INITIALIZER;

void *motor_update(void *arg)
{
    while (1)
    {
        pthread_mutex_lock(&encoder_mutex);
        encoder_value_R = encoder.pulse_counter_update();
        encoder_value_L = encoderL.pulse_counter_update();
        target_speed_R = target_speed + diff_speed / 2;
        target_speed_L = target_speed - diff_speed / 2;
        pthread_mutex_unlock(&encoder_mutex);

        control_speed_R = pid_calculate(&pid_R, encoder_value_R, target_speed_R);
        control_speed_L = pid_calculate(&pid_L, encoder_value_L, target_speed_L);
        motor_set_speed(motor, control_speed_R);
        motor_set_speed(motorL, control_speed_L);
        cout << "encoder:R" << encoder_value_R << "\tencoder:L" << encoder_value_L << "-----";
        cout << "L:" << control_speed_L << "\t R:" << control_speed_R;
        cout << "diff_speed:" << diff_speed << endl;

        usleep(5000); // 5ms更新一次电机PID
    }
}

void signal_handler(int sig)
{

    pthread_cancel(motor_thread); // 终止线程

    pwm_set_duty_cycle(pwm1, 0.45);     //直着停止

    usleep(25000);

    pwm_destroy(pwm1);

    writer.release();

    close(sock);        //关闭套接字

    motor_destroy(motor);
    motor_destroy(motorL);

    printf("程序终止！\n");

    cout << sig << " handled successfully." << endl;
    exit(0); // exit
}

void cleanup()
{
    pthread_cancel(motor_thread); // 终止线程

    pwm_set_duty_cycle(pwm1, 0.41);

    usleep(25000);

    pwm_destroy(pwm1);

    writer.release();

    motor_destroy(motor);
    motor_destroy(motorL);
    // 销毁互斥锁
    pthread_mutex_destroy(&encoder_mutex);
    printf("程序终止！\n");
    cout << "clean succeeded." << endl;
    exit(0); // exit
}

void terminateHandler()
{
    cout << "Program terminated unexpectedly. Cleaning up..." << endl;
    // 执行清理操作
    abort();
    return;
}

// 计算两点间向量角度差
double calcAngle(const Point &p1, const Point &p2, const Point &p3)
{
    Point v1 = p1 - p2; // 前向向量
    Point v2 = p3 - p2; // 后向向量

    double dotProd = v1.x * v2.x + v1.y * v2.y;
    double normV1 = sqrt(v1.x * v1.x + v1.y * v1.y);
    double normV2 = sqrt(v2.x * v2.x + v2.y * v2.y);

    if (normV1 == 0 || normV2 == 0)
        return 180.0; // 避免除零错误
    if (dotProd < 0)
        return 180; // 钝角
    double cosTheta = dotProd / (normV1 * normV2);
    return acos(cosTheta) * 180.0 / CV_PI; // 转换为角度
}

void detectAngles(const std::vector<Point> &boundary,
                  std::vector<Point> &rightAngles,
                  std::vector<Point> &acuteAngles,
                  Mat &mid,
                  int step = 3, // 点间距步长
                  double angleThreshold = 40.0)
{
    for (int i = 0; i < boundary.size() - step; i += step)
    {
        int j = 0, k = 0;
        Point p1 = boundary[i];
        Point p2 = Point(-1, -1), p3 = Point(-1, -1);
        for (j = i + 1; j < boundary.size() - step; j++)
        {
            Point p = boundary[j];
            Point v1 = p1 - p; // 前向向量
            double normV1 = sqrt(v1.x * v1.x + v1.y * v1.y);
            if (normV1 <= 10 || normV1 >= 30)
                continue; // 模长太短或太长
            p2 = p;
            break;
        }

        for (k = j + 1; k < boundary.size() - step; j++)
        {
            Point p = boundary[j];
            Point v2 = p - p2; // 前向向量
            double normV1 = sqrt(v2.x * v2.x + v2.y * v2.y);
            if (normV1 <= 10 || normV1 >= 30)
                continue; // 模长太短或太长
            p3 = p;
            break;
        }

        if (p2.x == -1 || p3.x == -1 || p2.x >= IMG_WIDTH - 5 || p2.x <= 4)
            continue; // 没选到合适的p2 p3 点 或者检测出来的角点在边界上

        // if(p1.y>=IMG_HEIGHT/2||p2.y>=IMG_HEIGHT/2||p3.y>=IMG_HEIGHT/2)    continue;     //太高的点不需要找角

        double angle = calcAngle(p1, p2, p3);

        if (angle >= 20 && angle <= 60 || angle >= 80 && angle <= 100)
        { // 直角判定
            cout << p2.x << "," << p2.y << "  " << angle << endl;
            rightAngles.push_back(p2);
            circle(mid, p1, 2, Scalar(128, 0, 128), -1); // 红色标记直角
            circle(mid, p3, 2, Scalar(128, 0, 128), -1); // 红色标记直角
        }
        else if (angle < angleThreshold)
        { // 锐角判定
            acuteAngles.push_back(p2);
        }
    }
}

int main(void)
{ // atexit(cleanup);
    // std::set_terminate(terminateHandler);
    signal(SIGINT, signal_handler);                         // 注册Ctrl+C信号的处理函数
    signal(SIGSEGV, signal_handler);                        // 注册err 信号处理器
    signal(SIGABRT, signal_handler);                        // 注册abort 信号处理器
    signal(SIGBUS, signal_handler);                         // 注册Bus信号处理器
    pid_init(&pid_R, 2.15, 0.24, 0.78, 0.62, target_speed); // 参数分别为//kp,ki,kf,fi,target
    pid_init(&pid_L, 2.15, 0.24, 0.74, 0.7, target_speed);
    std::cout << "Encoder initialized." << std::endl;



    
    struct sockaddr_in serv_addr;

    // 创建套接字
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cerr << "创建套接字失败" << std::endl;
        raise(SIGINT);
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(1347); // 设置服务器端口号

    // 将服务器地址转换为二进制格式
    if (inet_pton(AF_INET, "192.168.137.1", &serv_addr.sin_addr) <= 0) { 
        std::cerr << "地址转换失败" << std::endl;
        raise(SIGINT);
    }

    // 连接到服务器
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cerr << "连接失败" << std::endl;
        raise(SIGINT);
    }


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

    int state = st_normal; // 初始状态下正常直行
    float motor_pwm = 0.45;

    // jzy servo
    pwm_set_frequency(pwm1, 300);
    pwm_set_duty_cycle(pwm1, motor_pwm);
    pwm_enable(pwm1);

    Mat img;

    vector<Point> judge_points = {}; // 前瞻点
    for (int i = DOWN_DETECT; i >= UP_DETECT; i--)
    {
        judge_points.push_back(Point(CAR_X, i));
    }

    int corner = 0;

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

        int gap = 0;

        int left_lost = 0, right_lost = 0; // 丢线情况

        vector<Point> left_border = {}, right_border = {}, mid_border = {}; // 赛道左右边界点 和中线 坐标

        vector<Point> max_white_col = {}; // 最长白列的点集

        vector<Point> right_angles = {}, acute_angles = {}; // 存储直角和锐角的点

        vector<Point> sm_left_border = {}, sm_right_border = {};

        Vec4f left_assess_line, right_assess_line; // 赛道左右丢线评估线

        flip(img,img,-1);            //垂直+水平翻转图像

        
        mid = img.clone();

        cvtColor(img, gray, COLOR_BGR2GRAY); // 灰度化

        // threshold(gray, bin, 0, 255,THRESH_BINARY | THRESH_OTSU);       //大津法 二值化
        Canny(gray, edge, 50, 150); // 边缘检测

        static int zebra_counter = 0;    // 斑马线连续检测计数器
        static int obstacle_counter = 0; // 障碍物连续检测计数器
        static float lane_width_avg = 0; // 车道宽度移动平均值

        Imfor info = get_border_points_and_get_gap(edge, left_border, right_border, mid_border, max_white_col);

        std::cout << "Gap: " << info.gap << std::endl;
        std::cout << "Distance: " << info.distance << std::endl;
        gap = info.gap;
        if (info.distance < 10 && info.distance > 0)
        {
            zebra_counter = 1;
        }
        else
            zebra_counter = 0;

        // gap=get_border_and_get_gap_neighbour8(edge,left_border,right_border,mid_border);        //获取赛道左右边界点 和中线 误差

        // detectAngles(left_border, right_angles, acute_angles,mid);
        // detectAngles(right_border, right_angles, acute_angles,mid);

        // for (auto& p : right_angles)
        //   circle(mid, p, 3, Scalar(128, 0, 128), -1); // 紫色标记直角
        // for (auto& p : acute_angles)
        // circle(mid, p, 3, Scalar(0,255,0), -1); // 绿色标记锐角

        left_assess_line = fit_assess_line(left_border); // 拟合评估线
        right_assess_line = fit_assess_line(right_border);

        left_lost = get_lost(left_assess_line, left_border);
        right_lost = get_lost(right_assess_line, right_border);

        if (zebra_counter >= 1)
        {
            state = st_zebra;
        }
        else
        {
            state = get_state(state, left_lost, right_lost, left_assess_line, right_assess_line);
            if (info.distance < 60 && info.distance > 20 && state == st_normal)
            {
                state = st_obstacle;
            }
        }

        state = get_state(state, left_lost, right_lost, left_assess_line, right_assess_line); // 获取状态
        switch (state)
        {               // 状态机 根据当前状态选择巡线策略
        case st_normal: // 正常状态
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        case st_Rcircle_1:                               // 直行但准备入环
            gap = get_single_gap(left_border, true, 60); // 左侧单侧巡线
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        case st_Rcircle_2:                                 // 入环+ 右侧单侧巡线
            gap = get_single_gap(right_border, false, 50); // 右侧单侧巡线
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        case st_Rcircle_3: // 单侧右巡线但左边出现丢线
            gap = get_single_gap(right_border, false, 50);
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;
        case st_Rcircle_4:
            gap = get_single_gap(left_border, true, 60);
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm);
            break;

        case st_zebra:        // 斑马线状态
            target_speed = 0; // 基准速度18 -> 完全停止
            raise(SIGINT);
        case st_obstacle: // 避障状态
            target_speed = 15;
            gap = get_single_gap(left_border, true, 20);
            motor_pwm = PID_Servo(steering_pid, (float)gap);
            pwm_set_duty_cycle(pwm1, motor_pwm); // 模仿出环岛
            break;
        default:
            cout << "------------------" << endl;
            cout << "-----意外状态------" << endl;
            cout << state << endl;
            raise(SIGSEGV);
            break;
        }

        cout << "LE:" << left_lost << "\tRE:" << right_lost << "\tS:" << state << endl;

        // 差速
        if (abs(gap) > DIFF_THRESHOLD)
        {
            // 计算转向比：基于gap超出阈值的部分
            float steer_ratio = (gap - (gap > 0 ? DIFF_THRESHOLD : -DIFF_THRESHOLD)) / 100.0f;
            diff_speed = DIFF_KP * target_speed * steer_ratio;
            // std::cout << "差速输出：Diff Speed: " << diff_speed << std::endl;
        }
        else
        {
            diff_speed = 0.0; // 小误差时禁用差速
        }
        if (diff_speed > 7.5)
            diff_speed = 7.5; // 限制超调
        // roundabout(gray,edge);

        /*
        Mat eight=img.clone();
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::Mat dilatedEdge;
        cv::dilate(edge, dilatedEdge, kernel);
    // 使用膨胀后的图像寻找赛道边界
        std::vector<cv::Point> boundary = findTrackBoundary(dilatedEdge);
        cout<<"eight num:"<<boundary.size()<<endl;
        for(int i=0;i<boundary.size();i++)
        {
            circle(eight,boundary[i],1,Scalar(255,0,0),-1);
        }
            */

        // control_servo((float)gap);
        // pi.compute((float)gap);
        // control_servo((float)gap);

        auto end1 = std::chrono::high_resolution_clock::now();
        // 处理完1帧图像
        std::chrono::duration<double, std::milli> elapsed = end1 - start;

        cout << elapsed.count() << "ms" << "\toutput:" << motor_pwm << "\tgap:" << gap << endl; // 处理1帧图像耗时
        
        char msg[20]="";
        sprintf(msg, "%d\n", gap);
        send(sock, msg, strlen(msg), 0);


    

        draw_by_points(left_border, mid, 0, 255, 0); // 绘制边界  绿色
        draw_by_points(right_border, mid, 0, 255, 0);
        draw_by_points(mid_border, mid, 255, 0, 0); // 绘制中线 红色

        draw_by_points(judge_points, mid, 0, 0, 255); // 前瞻点 蓝色

        // cout<<"max_white_col.size():"<<max_white_col.size()<<endl;
        draw_by_points(max_white_col, mid, 0, 255, 255); // 最长白列用青色标出

        drawline(mid, left_assess_line, 255, 165, 0); // 绘制橙色评估线
        drawline(mid, right_assess_line, 255, 165, 0);

        // 显示图像
        putText(mid, "LE:" + to_string(round(left_lost * 100) / 100), Point(5, 15), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);  // 左线误差
        putText(mid, "RE:" + to_string(round(right_lost * 100) / 100), Point(5, 25), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA); // 右线误差

        putText(mid, "g:" + to_string(gap), Point(80, 15), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "p:" + to_string(motor_pwm), Point(80, 25), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "S:" + to_string(state), Point(5, 35), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "R" + to_string(encoder_value_R), Point(80, 110), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "" + to_string(target_speed_R), Point(80, 120), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "L" + to_string(encoder_value_L), Point(5, 110), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        putText(mid, "" + to_string(target_speed_L), Point(5, 120), FONT_HERSHEY_PLAIN, 0.8, Scalar(0, 165, 255), 1, LINE_AA);
        writer.write(mid); // 写入视频文件

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

        auto end2 = std::chrono::high_resolution_clock::now();
        elapsed = end2 - end1; // debug耗时
        cout << "debug:" << elapsed.count() << "ms" << endl;
        ;
    }

    // sleep(50);
    // pwm_destroy(pwm1);
    // pwm_destroy(pwm2);

    raise(SIGINT);
    return 0;
}
