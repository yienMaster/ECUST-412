/*
@author 杳泽
@brief 舵机控制、摄像头相关
*/



#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>
#include <string>
#include <chrono>
#include <thread>
#include <signal.h>
#include <stdio.h>
#include <math.h>


//自定义头文件
#include "gpio.h"
#include "pwm.h"
#include "servo.h"
#include "tools.h"

using namespace std;
using namespace cv;



const bool is_debug=false;      //调试开关

extern float pre_feed;       //前馈项


extern VideoWriter writer;      //声明视频编辑器
extern VideoWriter writer_canny;      //声明视频编辑器
extern VideoWriter writer_bird; 

const int UP_DETECT=70-15;        //前瞻点的最高坐标(y)
const int DOWN_DETECT=(UP_DETECT+5*2)  ;     //连取10个像素大小的点作为前瞻点   
// 10% 85~90


const int UP_BORDER=20;            //绘制的边界线和中线的最高和最低位置(y)
const int DOWN_BORDER=95;


extern int width_table[];       //宽度表     width_table[y] 为对于y的道宽


const int IMG_WIDTH=160;       //图像宽高
const int IMG_HEIGHT=120;


const int CAR_X=IMG_WIDTH/2;       //+7是修正车子图像的中点

const int UP_LIMIT=40;      //评估线的长度          评估线越长 越容易区分弯道和直到
//但也不能太长


const int ASSESS_POINTS_NUM=10;     //评估线的点集大小


//const int guide_gap=65;          //认为的 赛道中央到赛道边缘的距离   用于单线循迹 单位像素
//量前瞻点最底部位置的中线 x和同一y坐标下边缘点的x的差值


const int st_normal=0;      //直行 或正常弯道


const int st_cross=8;       //交叉路口


//右圆环     1-5
const int st_Rcircle_1=1;       //圆环1阶段(一侧直线一侧丢线->继续直行)
const int st_Rcircle_2=2;    // 圆环2阶段   单侧右巡线
const int st_Rcircle_3=3;    // 圆环3阶段   单侧右巡线且左边出现丢线
const int st_Rcircle_4=4;    // 圆环4阶段   单侧左巡线
const int st_Rcircle_5=5;    // 圆环5阶段   出环
const int  st_zebra=6  ;   // 新增：斑马线状态
const int st_left_obstacle=7  ;   // 新增：避障状态
const int st_right_obstacle=9  ;   // 新增：避障状态

extern bool is_right_circle;   //是否为右圆环


struct Imfor {
    int gap;
    int distance;
    int distance2;
};
extern Imfor info;
extern int last_distance2 ;
extern int last_distance ;




const int miss_lost=1300;       //丢线阈值
const int turn_lost=250 ;           //弯道阈值  <250 直道  >250 弯道


const int state_interval=4000;      //状态切换时间间隔  ms


extern Mat img;


// 定义八邻域偏移量
const Point neighbors8[8] = {
    Point(-1, -1), Point(0, -1), Point(1, -1),
    Point(-1, 0),               Point(1, 0),
    Point(-1, 1), Point(0, 1), Point(1, 1)
};




extern float diff_speed ;            // 动态差速值
extern const float DIFF_THRESHOLD ; // 差速激活阈值
extern const float DIFF_KP ;        // 0.33;   // 差速比例系数
extern const float diff_limit;      //   // 差速限制




// PID 控制结构体    电机
struct PID      //电机
{
    double target;          // 目标值
    double kp;              // 比例系数
    double ki;              // 积分系数
    double kd;              // 微分系数
    double integral;        // 积分项
    double last_error;      // 上次误差
    double kf;              // 前馈系数
    //double filter_alpha;    // 新增输出滤波系数 [0-1]
    //double filtered_output; // 新增滤波状态
};

/*
@brief PI控制计算函数，执行顺序：死区处理→滤波→限幅 
@param PID *pid  PID结构体
@param double feedback 反馈值
@param double target 目标值
@return double 输出值
*/
double pid_calculate(PID *pid, double feedback, double target);


/*
@brief 初始化PID
@param PID *pid  PID结构体
@param double kp 比例系数
@param double ki 积分系数
@param double kf 前馈系数
@param double fi 输出滤波系数
@param double target 目标值
*/
void pid_init(PID *pid, double kp, double ki, double kf, double fi, double target);




//非自适应PID    舵机
/**/
struct PIDController {    //舵机
    // p < 0.013   <=0.005   <0.007   <0.006
    //i  <0.2   <0.1   <0.05   <0.005
    /*
    float Kp = 0.005f;   // 比例系数（需调试）
    float Ki = 0.00f; // 积分系数（需调试） 
    float Kd = 0.00f;  // 微分系数（需调试）
    */
    // 0.0030  0.0005在26ok
    //速度20   80.0
    float Kp = 0.0058f;   // 比例系数（需调试）   //0.0065  0.0073 偏大
    float Ki = 0.00f; // 积分系数（需调试） 
    float Kd = 0.0005f;  // 微分系数（需调试）    //0.0005
    //25/0503    @ 20 70-15; 10 rows  0.0046   0.0005


    //0.013   0  0.001   
    //0.009   0.001
    //0.0006 70-20 擦赛道  /0.00065 @27


    //已调试出的合适   PID_servo 综合输出那里/80.0f   速度 10
    // P=0.02   K=0.015 D=0.0013
    // 0.025  0.01  0.0013
    //0.03 0.01 0.0013轻微震荡
    // 运行参数
    float integral = 0.0f;      // 积分累计
    float prev_error = 0.0f;    // 上一次误差
    float prev_time = 0.0f;     // 上一次时间戳
    float output = 0.45f;       // 初始化直行PWM
    
    // 系统参数
    const float PWM_MIN = 0.41f; // 最大右转
    const float PWM_MAX = 0.49f; // 最大左转
    const float PWM_CENTER = 0.45f;


     // 新增前馈参数
     float Kf = 0.000f;          // 前馈增益系数（需调试）
     float steering_rate = 0.0f; // 舵机角速度估计
     float prev_steering = 0.45f; // 前次舵机指令
     float rate_filter = 0.1f;   // 角速度低通滤波系数（0.1-0.3）

};


/*
@brief 舵机PID
@param PIDController& pid  PID控制器
@param float current_error 当前误差
@return float 输出PWM
*/
float PID_Servo(PIDController& pid, float current_error) ;



/*
@brief 保存图片到本地
@param  Mat 图片
@param  string  文件名
*/
void save_pic(Mat pic,string name);


/*
@brief 寻找赛道左右边界点
@param Mat canny 后的图片
@param vector<Point>& 左边界点向量
@param vector<Point>& 右边界点向量
@param vector<Point>& 中线点向量
@param vector<Point>& 最长白列的点集
*/



/*
@brief 寻找赛道左右边界点
@param Mat canny 后的图片
@param vector<Point>& 左边界点向量
@param vector<Point>& 右边界点向量
@param vector<Point>& 中线点向量
@param vector<Point>& 最长白列的点集
@param Mat bin 二值化后的图片  用于判断赛道
@parm struct lost  丢线结构体
@return Imfor 结构体  包含gap和distance
*/
Imfor get_border_points_and_get_gap(const Mat& edgeImage, std::vector<Point>& left_border, std::vector<Point>& right_border,
    std::vector<Point>& mid_border, std::vector<Point>& max_white_col,const Mat& bin,
    struct cross_lost& lost_point,
    std::vector<Point> &left_valid_border, std::vector<Point> &right_valid_border) ;
/*
@brief 根据点集绘制轨迹,常见RGB值 黑(0,0,0) 红(255,0,0) 蓝(0,0,255) 绿(0,255,0) 橙(0,165,255)
@param const vector<Point> 点集
@param Mat& 带有轨迹的赛道图像
@param RGB int值 ,用于指定轨迹的颜色
*/
void draw_by_points(const vector<Point> points,Mat& pic,const int R,const int G,const int B);

/*
@brief 处理Ctrl + C信号
@param int sig  信号类型
*/
void handle_sigint(int sig) ;

/*
@brief PI
@param int current
@param int target
@return pwm输出
*/
float Incremental_PI(int current, int target); // PI


/*
@brief 设置摄像头参数
@param 要设置的摄像头
*/
void set_cam(VideoCapture cam);



/*
@brief 根据偏差控制舵机
@return 舵机pwm信号 0.41~0.49;
*/
float control_servo(const float gap);





/*
@brief  计算丢线情况,从前瞻点最底部开始到 前瞻点最上部15个像素      总共20个像素与同y值 评估线点的差
@param const Vec4f 评估线
@paran const vector<Point> 边界点集
@return int 丢线情况
*/
int get_lost(const Vec4f line,const vector<Point> points);


/*
@brief 传入边界点集,并利用最下侧的10个点（非丢线点）拟合评估线
@param const vector<Point> 点集
@return Vec4f 评估线
*/
Vec4f fit_assess_line(const vector<Point> points);


/*
@brief  画出Vec4f类型的直线并计算丢线情况
@param cv::Mat& 图像
@param const cv::Vec4f 线
@param const int R      颜色
@param const int G
@param const int B
*/
void drawline(cv::Mat& image, const cv::Vec4f& line, const int R,const int G,const int B) ;



/*
@brief 单侧巡线误差估计
@param vector<Point> 单侧边界点集
@param bool 传入的是否是左侧边界
@param const int 赛道中央到赛道边缘的距离(假定的)
@return int gap
*/
int get_single_gap(vector<Point> single_points,const bool is_left,const int guide_distance);



/*
@brief 附带前馈补偿的舵机PID
@param PIDController& pid  PID控制器
@param float current_error 当前误差
@return float 输出PWM
*/ 
float PID_Servo_WithDelayComp(PIDController& pid, float current_error) ;





/*
@brief 计算3点间两向量的角度
@param const Point &p1  点1
@param const Point &p2  点2
@param const Point &p3  点3
@return double 向量21,向量23的角度
*/
double calcAngle(const Point &p1, const Point &p2, const Point &p3);


//void get_bird_view(const Mat& src, Mat& dst, const Point2f& src_points[4], const Point2f& dst_points[4]);


/*
@brief 状态机
@param const int current_state 当前状态
@param const vector<Point> left_border  左边界点集
@param const vector<Point> right_border  右边界点集
@param const vector<Point> left_lost  左丢线点集
@param const vector<Point> right_lost  右丢线点集
@param vector<Point>& corners 角点
@return int 状态
*/
int get_state(const int current_state,
    const vector<Point> left_border, const vector<Point> right_border,
    const vector<Point> left_lost, const vector<Point> right_lost,
    vector<Point>& corners);



/*
@brief 十字路口角点检测
@param const vector<Point> left_border  左边界点集
@param const vector<Point> right_border  右边界点集
@param const struct cross_lost lost_point  丢线结构体
@param Point& left_down_corner  左下角点
@param Point& left_up_corner  左上角点
@param Point& right_down_corner  右下角点
@param Point& right_up_corner  右上角点
@return void
*/
void cross_corner_detect(const vector<Point> left_border,const vector<Point> right_border,
        const struct cross_lost lost_point,
        Point& left_down_corner,Point& left_up_corner,Point& right_down_corner,Point& right_up_corner);


/*
@brief 单侧角点检测
@param const vector<Point> left_border  左边界点集
@param const vector<Point> right_border  右边界点集
@param const struct cross_lost lost_point  丢线结构体
@param vector<Point> corners 角点
@param const bool is_right  是否为右侧

@return void
*/
void single_corner_detect(
    const vector<Point>& left_border,   // 修复1：改为引用传递，避免大对象拷贝导致的栈溢出
    const vector<Point>& right_border,  
    const vector<Point>& left_lost,     
    const vector<Point>& right_lost,    
    vector<Point>& corners, 
    const bool is_right
) ;

/*
@brief 单侧角点检测,不严格的,从上到下仅利用边界一致性检测
@param const vector<Point> left_border  左边界点集
@param const vector<Point> right_border  右边界点集
@param const struct cross_lost lost_point  丢线结构体
@param vector<Point> corners 角点
@param const bool is_right  是否为右侧

@return void
*/
void single_corner_detect_no_vec(const vector<Point> left_border, const vector<Point> right_border,
    const vector<Point> left_lost, const vector<Point> right_lost,
    vector<Point>& corners, const bool is_right);


    /*
@brief 单侧角点检测,不严格的,从下到上仅利用向量法角度检测
@param const vector<Point> left_border  左边界点集
@param const vector<Point> right_border  右边界点集
@param const struct cross_lost lost_point  丢线结构体
@param vector<Point> corners 角点
@param const bool is_right  是否为右侧

@return void
*/
void single_corner_detect_no_dist(const vector<Point> left_border, const vector<Point> right_border,
        const vector<Point> left_lost, const vector<Point> right_lost,
        vector<Point>& corners, const bool is_right);

/*
@brief 计算两个向量的角度
@param const Point v1  向量1
@param const Point v2  向量2
@return double 角度
*/
    double calcAngle_v(const Point v1, const Point v2);


struct cross_lost{
    //十字丢线结构体

    //!!!----------y_min 仅统计从下往上第一段丢线点的最小y值----------!!!
    int L_y_max;
    int L_y_min;        //做丢线点集坐标最大y和最小y
   // int L_x_min;  
   // int L_x_max;

    int R_y_max;        //右
    int R_y_min;
    //int R_x_min;  
    //int R_x_max;


    vector<Point> left;     //左丢线点集
    vector<Point> right;        //右丢线点集
};


/*
@brief 多项式拟合中线曲线
@param 中线点,多项式次数,拟合后的系数
@return bool 是否拟合成功
*/
bool polynomial_curve_fit_y(std::vector<cv::Point>& points, int order, cv::Mat& coeff) ;


/*
@brief 根据拟合的曲线获得坐标点
@param 拟合后的系数
@param 拟合点y的最小值
@param 拟合点y的最大值
@return 拟合后的点集


*/
std::vector<cv::Point> generate_fitted_points_y(const cv::Mat& coeff, int y_min, int y_max) ;


bool detect_st_obstacle(const Mat &edgeImage, std::vector<Point> &left_border, std::vector<Point> &right_border, const Mat &bin,
    struct cross_lost &lost_point,
    std::vector<Point> &left_valid_border, std::vector<Point> &right_valid_border);


    
/*
@brief 计算各个角点之间距离的75%分位数
@param const vector<Point> 角点集合
@return int 75%分位数
*/
int dist_75(const vector<Point> corners);


/*
@brief 当单侧丢线严重时的修正后中线,以及相应的gap
@param const vector<Point> border 单侧边界点集 
@param const bool is_right 传入的单侧边界点集是否是右侧
@param vector<Point>& mid_c  修正中线vector
@param const int gap 原来的gap
@return int gap     
*/
int correct_mid(const vector<Point> border,const bool is_right,vector<Point>& mid_c,const int gap);


/*
@brief 根据基准宽度幅值宽度表
*/
void give_width();



/*
@brief 边界连续性检测
@param const vector<Point> border 赛道边界
@param const float threshold 连续性阈值
@return bool 是否连续性
*/
bool is_border_continous(const vector<Point> border,const float threshold);


/*
@brief 当前的gap计算是否是依照有效边界计算的 (也就是当前前瞻点同等高度是否是有效边界)
@param const vector<Point> border  边界
@return true/false
*/
bool is_valid_gap(const vector<Point> border);
