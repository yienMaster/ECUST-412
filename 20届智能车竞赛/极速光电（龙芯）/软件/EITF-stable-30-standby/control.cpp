/*
@author 杳泽
@brief 舵机控制、摄像头相关
*/

#include "control.h"




// PID计算函数
float PID_Servo(PIDController &pid, float current_error)
{
    // 计算时间差（秒）

    auto now = std::chrono::steady_clock::now();
    float time = std::chrono::duration<float>(now.time_since_epoch()).count();
    // if(current_error<=4) return 0.45;
    float dt = time - pid.prev_time;
    pid.prev_time = time;

    // 比例项
    float P = pid.Kp * current_error;

    // 积分项（带抗饱和）
    pid.integral += current_error * dt;
    pid.integral = (pid.integral < -100.0f) ? -100.0f : (pid.integral > 100.0f) ? 100.0f
                                                                                : pid.integral;
    // 限制积分积累

    float I = pid.Ki * pid.integral;

    // 微分项
    float derivative = (current_error - pid.prev_error) / dt;

    // cout<<"err rate:"<<derivative<<endl;
    float D = pid.Kd * derivative;

    // 综合输出
    pid.output = pid.PWM_CENTER + (P + I + D) / 80.0f; // 将误差映射到PWM范围

    // 输出限幅
    pid.output = (pid.output < pid.PWM_MIN) ? pid.PWM_MIN : (pid.output > pid.PWM_MAX) ? pid.PWM_MAX
                                                                                       : pid.output;

    // 保存误差
    pid.prev_error = current_error;

    return pid.output;
}

// 绘制直线的函数
void drawline(cv::Mat &image, const cv::Vec4f &line, const int R, const int G, const int B)
{
    if (line[0] == 0) // 不能除0
    {
        for (int i = line[3]; i > UP_DETECT - UP_LIMIT; i--) // 直接绘制垂直的评估线
        {
            image.at<Vec3b>(i, line[2])[0] = B; // B
            image.at<Vec3b>(i, line[2])[1] = G; // G
            image.at<Vec3b>(i, line[2])[2] = R; // R
        }
        return;
    } // 斜率不存在 直线垂直于y轴

    float k = line[1] / line[0];     // 斜率
    float b = line[3] - k * line[2]; // 截距
                                     // 获取边线上 前瞻点y 及前瞻点y上方15个像素的丢线误差

    int x = -1;
    for (int i = DOWN_DETECT; i > UP_DETECT - UP_LIMIT; i--) // 绘制评估线
    {
        x = (i - b) / k;
        if (x < 0 || x > IMG_WIDTH)
            continue;                 // x坐标越界
        image.at<Vec3b>(i, x)[0] = B; // B
        image.at<Vec3b>(i, x)[1] = G; // G
        image.at<Vec3b>(i, x)[2] = R; // R
    }

    return;
}

void draw_by_points(const vector<Point> points, Mat &pic, const int R, const int G, const int B)
{
    bool wrong_RGB = false;

    if (R > 255 || R < 0 || G > 255 || G < 0 || B > 255 || B < 0)
        wrong_RGB = true; // RGB值错误

    if (wrong_RGB) // RGB值错误 画黑线
    {
        for (int i = 0; i < points.size(); i++)
        {
            pic.at<Vec3b>(points[i].y, points[i].x)[0] = 0; // B
            pic.at<Vec3b>(points[i].y, points[i].x)[1] = 0; // G
            pic.at<Vec3b>(points[i].y, points[i].x)[2] = 0; // R
        }
    }
    else // 颜色没错
    {
        for (int i = 0; i < points.size(); i++)
        {
            pic.at<Vec3b>(points[i].y, points[i].x)[0] = B; // B
            pic.at<Vec3b>(points[i].y, points[i].x)[1] = G; // G
            pic.at<Vec3b>(points[i].y, points[i].x)[2] = R; // R
        }
    }
    return;
}

bool polynomial_curve_fit_y(std::vector<cv::Point>& points, int order, cv::Mat& coeff) {
    int N = points.size();
    cv::Mat X = cv::Mat::zeros(order + 1, order + 1, CV_64F);
    cv::Mat Y = cv::Mat::zeros(order + 1, 1, CV_64F);

    // 构造X矩阵（y的幂次项）
    for (int i = 0; i <= order; ++i) {
        for (int j = 0; j <= order; ++j) {
            for (int k = 0; k < N; ++k) {
                X.at<double>(i, j) += std::pow(points[k].y, i + j); // 使用y的幂次
            }
        }
    }

    // 构造Y矩阵（x作为目标值）
    for (int i = 0; i <= order; ++i) {
        for (int k = 0; k < N; ++k) {
            Y.at<double>(i, 0) += std::pow(points[k].y, i) * points[k].x; // 计算x的线性组合
        }
    }

    // 求解系数
    cv::solve(X, Y, coeff, cv::DECOMP_SVD); // 使用SVD提高稳定性
    return true;
}


std::vector<cv::Point> generate_fitted_points_y(const cv::Mat& coeff, int y_min, int y_max) {
    std::vector<cv::Point> fitted_points;
    for (int y = y_min; y <= y_max; ++y) {
        double x = 0;
        for (int i = 0; i < coeff.rows; ++i) {
            x += coeff.at<double>(i, 0) * std::pow(y, i); // 累加多项式项
        }
        fitted_points.emplace_back(cv::Point(x, y));
    }
    return fitted_points;
}










Imfor get_border_points_and_get_gap(const Mat &edgeImage, std::vector<Point> &left_border, std::vector<Point> &right_border,
                                    std::vector<Point> &mid_border, std::vector<Point> &max_white_col, const Mat &bin,
                                    struct cross_lost &lost_point,
                                    std::vector<Point> &left_valid_border, std::vector<Point> &right_valid_border)
{
    const int imageHeight = edgeImage.rows; // 图像高宽
    const int imageWidth = edgeImage.cols;

    const int reserve_height = 10; //底部保留的像素高度(不巡线的部分)

    vector<Point> &left_lost = lost_point.left;
    vector<Point> &right_lost = lost_point.right;

    int gap = 0; // 中线误差
    int total_distance = 0;
    int valid_lines = 0;
    int total_distance2 = 0;
    int valid_lines2 = 0;

    int right_max_x = imageWidth / 2, left_max_x = imageWidth / 2; // 左半边和右半边最长白列的 x 坐标
    int max_x = -1;                                                // 最长白列的 x 坐标
    int right_max_length = 0, left_max_length = 0;                 // 左右最长白列的长度

    // 从底部中间向左寻找最长白列
    for (int x = imageWidth / 2; x >= 0; x--)
    {
        if (bin.at<uchar>(imageHeight - reserve_height, x) == 0)
            continue;   // 如果不是赛道 则跳过
        int length = 0; // 当前 x 坐标白列长度
        for (int y = imageHeight - reserve_height; y >= 0; y--)
        {
            if (edgeImage.at<uchar>(y, x) == 0)
            { // 白列还未终止
                length++;
            }
            else
            {
                break; // 白列已终止
            }
        }
        if (length >= left_max_length)
        {
            left_max_length = length; // 找到更长的白列
            left_max_x = x;
        }
        else
        {
            if (left_max_length - length > 5) // 这他妈肯定是单调减吧
                break;                        // 理论上最长白列的长度在左右都是单调的  白列开始变短则当前的最长白列就是最长白列  给予5像素容差
        }
    }

    // 从底部中间向右寻找最长白列
    for (int x = imageWidth / 2; x < imageWidth; x++)
    {
        if (bin.at<uchar>(imageHeight - reserve_height, x) == 0)
            continue;   // 如果不是赛道 则跳过
        int length = 0; // 当前 x 坐标白列长度
        for (int y = imageHeight - reserve_height; y >= 0; y--)
        {
            if (edgeImage.at<uchar>(y, x) == 0)
            { // 白列还未终止
                length++;
            }
            else
            {
                break; // 白列已终止
            }
        }
        if (length >= right_max_length)
        {
            right_max_length = length; // 找到更长的白列
            right_max_x = x;
        }
        else
        {
            if (right_max_length - length > 5)
                break; // 理论上最长白列的长度在左右都是单调的  白列开始变短则当前的最长白列就是最长白列   给予5像素容差
        }
    }

    // 取左右最长白列中 长度更大的那个
    max_x = (left_max_length > right_max_length) ? left_max_x : right_max_x;

    // 推入最长白列的所有点
    for (int y = imageHeight - reserve_height; edgeImage.at<uchar>(y, max_x) == 0; y--)
    {
        max_white_col.push_back(Point(max_x, y));
    }

    // 没找到最长白列 则默认从中间开始向外查找边界
    if (max_x < 0)
    {
        max_x = imageWidth / 2;
    }

    // 大大取大
    int min_y = max_white_col.back().y > UP_BORDER ? max_white_col.back().y : UP_BORDER;

    vector<array<int, 3>> L_y = {{-1, -1, 0}, {-1, -1, 0}, {-1, -1, 0}}, R_y = {{-1, -1, 0}, {-1, -1, 0}, {-1, -1, 0}}; // 左右边界的y坐标

    // y_max 值 ,y_min,  count
    int L_index = 0, R_index = 0; // L_y_min/L_y_max/R_y_min/R_y_max 索引

    // 遍历每一行，找到左右边界
    for (int y = DOWN_BORDER - 10; y > min_y; y--)
    {
        bool foundLeft = false; // 该行的左右边界是否被找到
        bool foundRight = false;

        // 从最长白列所在 x 坐标 向左查找左边界
        for (int x = max_x; x >= 0; x--)
        {
            if (edgeImage.at<uchar>(y, x) > 0)
            {
                left_border.push_back(Point(x, y));       // 推入 左边界点 vector
                left_valid_border.push_back(Point(x, y)); // 有效边界
                foundLeft = true;
                break;
            }
        }
        if (!foundLeft)
        {
            left_border.push_back(Point(0, y)); // 没找到,指定图像最左侧为左边界
            left_lost.push_back(Point(0, y));   // 同时也是丢线点
            if (L_index >= L_y.size())
                L_y.push_back({-1, -1, 0}); // 扩容

            if (L_y[L_index][0] < 0)
            {
                L_y[L_index][0] = y; // 该丢线线段的最大y
                L_y[L_index][1] = y; // 假定的最小y
            }
            else
            {
                if (abs(y - L_y[L_index][1]) > 2)
                    L_index++; // 该丢线线段结束
                else
                {
                    if (y < L_y[L_index][1])
                        L_y[L_index][1] = y; // 更新该丢线线段的最小y
                    L_y[L_index][2]++;       // 该丢线线段长度++
                }
            }

            // 理论上从下网上找, left_lost[0]为y坐标最低点,left_lost[left_lost.size()-1]为y坐标最高点
        }

        // 从中间向外查找右边界
        for (int x = max_x; x <= imageWidth - 1; x++)
        {
            if (edgeImage.at<uchar>(y, x) > 0)
            {
                right_border.push_back(Point(x, y));       // 推入 右边界点 vector
                right_valid_border.push_back(Point(x, y)); // 有效边界
                foundRight = true;
                break;
            }
        }
        if (!foundRight)
        {
            right_border.push_back(Point(imageWidth - 1, y)); // 没找到,指定图像最右侧为右边界
            right_lost.push_back(Point(imageWidth - 1, y));   // 同时也是丢线点

            if (R_index >= R_y.size())
                R_y.push_back({-1, -1, 0}); // 扩容

            if (R_y[R_index][0] < 0)
            {
                R_y[R_index][0] = y; // 该丢线线段的最大y
                R_y[R_index][1] = y; // 假定的最小y
            }
            else
            {
                if (abs(y - R_y[R_index][1]) > 2)
                    R_index++; // 该丢线线段结束
                else
                {
                    if (y < R_y[R_index][1])
                        R_y[R_index][1] = y; // 更新该丢线线段的最小y
                    R_y[R_index][2]++;       // 该丢线线段长度++
                }
            }
            // 理论上从下网上找, right_lost[0]为y坐标最低点,right_lost[right_lost.size()-1]为y坐标最高点
        }

        // 计算中线点坐标
        const int midX = (left_border.back().x + right_border.back().x) / 2.0; // 直接两边相加/2得中线


        mid_border.push_back(Point(midX, y));
        // 在检测区域内 计算 gap
        if (y >= UP_DETECT && y <= DOWN_DETECT)
        {
            gap += CAR_X - midX;
            total_distance2 += right_border.back().x - left_border.back().x;
             valid_lines2++;
            
        }
        if(y>=43&&y<=45)//高处
        {
        total_distance += right_border.back().x - left_border.back().x;
        valid_lines++;
        }
    }


    if (mid_border.back().y > UP_DETECT) // 如果最高点比UP_DETECT低 说明前瞻点出界了
    {
        /*
        for (int i = 0; i < 5; i++)
        {
            gap += CAR_X - mid_border[mid_border.size() - i - 1].x; // 取最高的5个点的x坐标差的之和 作为gap  拯救前瞻点出界
        }*/
        cv::Mat coeff;
        polynomial_curve_fit_y(mid_border, 2, coeff); // 2次多项式拟合
        gap=0;

        // 生成拟合曲线（假设y范围在UP_DETECT到DOWN_DETECT之间）
        auto fitted_points = generate_fitted_points_y(coeff, UP_DETECT, DOWN_DETECT);
        //cout<<fitted_points<<endl;
        for(int y=DOWN_DETECT,i=0;y>=UP_DETECT;y--,i++)
        gap+=CAR_X-fitted_points[i].x;
    }

    int max_count = 0, max_h = 0;
    //-------左丢线-----------
    for (int h = 0; h < L_y.size(); h++) // 找出最长的丢线线段 将其最高和最低y作为 y_min和y_max
    {
        if (L_y[h][2] > max_count)
        {
            max_count = L_y[h][2];
            max_h = h;
        }
    }
    lost_point.L_y_min = L_y[max_h][1];
    lost_point.L_y_max = L_y[max_h][0];

    //-------右丢线-----------
    max_count = 0, max_h = 0;
    for (int h = 0; h < R_y.size(); h++) // 找出最长的丢线线段 将其最高和最低y作为 y_min和y_max
    {
        if (R_y[h][2] > max_count)
        {
            max_count = R_y[h][2];
            max_h = h;
        }
    }
    lost_point.R_y_min = R_y[max_h][1];
    lost_point.R_y_max = R_y[max_h][0];

    int distance = (valid_lines > 0) ? total_distance / valid_lines : 0;
    int distance2=(valid_lines2>0)? total_distance2/valid_lines2:0;

    Imfor result;
    result.gap = gap;
    result.distance = distance;
    result.distance2=distance2;
    return result;
}

void save_pic(Mat pic, string name)
{
    bool isSuccess = imwrite(name, pic);
    if (!isSuccess)
    {
        cerr << "图像保存失败" << endl;
    }
}

Vec4f fit_assess_line(const vector<Point> points)
{
    Vec4f assess_line;
    vector<Point> assess_points; // 6个评估点

    if (points.size() < ASSESS_POINTS_NUM)
    {
        assess_line[0] = 0;
        assess_line[1] = 1;
        assess_line[2] = IMG_WIDTH - 1;
        assess_line[3] = DOWN_DETECT;
        return assess_line;
    } // 评估点不足6个 返回最右边的边缘 (会导致评估丢线值非常大)

    for (int k = 0, i = 0; k < points.size(); k++) // 间隔取点则 k+=2
    {
        if (points[k].x > 0 && points[k].x < IMG_WIDTH - 1 && points[k].y < IMG_HEIGHT - 1) // 边缘点不能作为评估点
        {
            assess_points.push_back(points[k]);
            i++;
        }
        if (i == ASSESS_POINTS_NUM)
            break;
    }

    if (assess_points.size() < ASSESS_POINTS_NUM)
    {
        assess_line[0] = 0;
        assess_line[1] = 1;
        assess_line[2] = IMG_WIDTH - 1;
        assess_line[3] = DOWN_DETECT;
        return assess_line;
    }

    fitLine(assess_points, assess_line, DIST_L2, 0, 0, 0); // 拟合评估线
    return assess_line;
}

/*
@brief PI
@param int current
@param int target
@return pwm输出
*/
float Incremental_PI(int current, int target) // PI
{
    const double Velocity_KP = 0.7, Velocity_KI = 0.0002, Velocity_KD = 0.001; // KP,KI,KD值       P——调整力度  I ——抑制静差  D——抑制震荡
    // cb  1 0.8  0.1
    //   1.4 0.8 0.15
    //  1 1 0.15          1 1 0.2        1 1 0
    static double Bias, output, Last_bias, Last2_bias; // 当前偏差,output,上一次偏差,上上次偏差

    Bias = target - current; // 自动整型提升 无需类型转换                  // 计算当前偏差
    if (fabs(Bias) < 10)
        return 0.45;                                                                                                     // 直行
    output += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias + Velocity_KD * (Bias - Last2_bias + 2 * Last_bias); // 增量式PID控制器
    Last2_bias = Last_bias;                                                                                              // 保存上上次偏差
    Last_bias = Bias;
    cout << "   output:" << output << endl; // 保存上一次偏差
    float pwm = output * 0.01;
    if (pwm > 0.49)
    {
        pwm = 0.49;
    }
    else if (pwm < 0.41)
    {
        pwm = 0.41;
    }
    return pwm; // 增量输出
}

float control_servo(const float gap)
{
    // 0.41  极限右转
    if (fabs(gap) < 2) // 死区
        return 0.45;
    if (gap <= -80)
        return 0.41; //
    if (gap >= 80)
        return 0.49;
    float ratio = gap / 80.0;
    float pwm = 0.45 + ratio * 0.05 * (1 + 0.01 * fabs(ratio)); // 非线性增益
    if (pwm < 0.41)
        pwm = 0.41;
    else if (pwm > 0.49)
        pwm = 0.49;
    return pwm;
}
//<0.1

void set_cam(VideoCapture cam)
{
    const int video_fps = 24, cam_fps = 120;
    int fourcc = VideoWriter::fourcc('M', 'J', 'P', 'G'); // 设置图片格式为MJPG
    cam.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cam.set(CAP_PROP_FPS, cam_fps); // 设置摄像头帧率为120
    cam.set(CAP_PROP_FRAME_WIDTH, 160);
    cam.set(CAP_PROP_FRAME_HEIGHT, 120); // 分辨率 160x120


    cout << "image size:" << cam.get(CAP_PROP_FRAME_WIDTH) << "x" << cam.get(CAP_PROP_FRAME_HEIGHT) << endl;
    cout << "fps:" << cam.get(CAP_PROP_FPS) << endl;

    const String output_file = "output.avi";

    if(is_debug)
    {
    writer.open(output_file, fourcc, video_fps, Size(160, 120)); // 实时保存视频为output.avi

    writer_canny.open("canny.avi", fourcc, video_fps, Size(160, 120)); // 实时保存视频为output.avi

    writer_bird.open("bird.avi", fourcc, video_fps, Size(160, 120));

    if (!writer.isOpened() || !writer_canny.isOpened() || !writer_bird.isOpened())
    {
        std::cerr << "视频文件创建失败" << std::endl;
        raise(SIGINT);
    }
    }
    
}

int get_lost(const Vec4f line, const vector<Point> points)
{
    int lost = 0;
    // 获取直线点斜式
    float k = line[1] / line[0];     // 斜率
    float b = line[3] - k * line[2]; // 截距

    int up_limit = UP_LIMIT; // 获取边线上 前瞻点y 及前瞻点y上方20个像素的丢线误差

    for (int i = DOWN_DETECT; i > UP_DETECT - up_limit && UP_DETECT - up_limit > 10; i--)
    {
        Point point_on_line = Point((i - b) / k, i); // 直线上的点

        bool found_same_y_point = false;
        // 找到y=i的点
        for (const auto &point : points)
        {
            if (point.y == i)
            {
                lost += abs(point.x - point_on_line.x); // 累加丢线误差
                found_same_y_point = true;
                break;
            }
            // cout<<"in"<<endl;
        }
        if (!found_same_y_point)
            up_limit++; // 往上多找一个点
        // cout<<"out:"<<i<<"\t"<<(i>UP_DETECT-up_limit)<<"\t"<<UP_DETECT-up_limit<<endl;
    }
    return lost;
}



int get_single_gap(vector<Point> single_points, bool is_left, const int guide_distance)
{
    int gap = 0;
    int midX = 0;
    for (int i = 0, j = 0; i < single_points.size() && j < DOWN_DETECT - UP_DETECT + 1; i++)
    {
        Point point = single_points[i];
        if (point.y <= DOWN_DETECT && point.y >= UP_DETECT) // 前瞻点同一y坐标
        {
            if (is_left) // 传入左侧边界
            {
                midX = point.x + guide_distance - j; // 计算当前中线x
            }
            else // 右侧边界
            {
                midX = point.x - guide_distance - j;
            }
            gap += CAR_X - midX; // 累计单侧误差
            j++;
        }
    }
    return gap;
}

float PID_Servo_WithDelayComp(PIDController &pid, float current_error)
{
    // 时间计算
    auto now = std::chrono::steady_clock::now();
    float time = std::chrono::duration<float>(now.time_since_epoch()).count();
    float dt = time - pid.prev_time;
    pid.prev_time = time;

    // 1. 基本PD计算
    float P = pid.Kp * current_error;
    float D = pid.Kd * (current_error - pid.prev_error) / dt;

    // 2. 机械延迟前馈补偿
    // 计算舵机角速度（低通滤波处理）
    float current_rate = (pid.prev_steering - pid.output) / dt; // 注意符号
    pid.steering_rate = pid.rate_filter * current_rate +
                        (1 - pid.rate_filter) * pid.steering_rate;

    // 前馈项 = 预估延迟 * 角速度
    float feedforward = pid.Kf * pid.steering_rate;

    pre_feed = feedforward; // 获取前馈项
    // 3. 综合输出
    pid.output = pid.PWM_CENTER + (P + D + feedforward) / 80.0f;

    // 4. 更新状态
    pid.prev_steering = pid.output; // 记录本次指令
    pid.prev_error = current_error;

    // 输出限幅
    pid.output = fmax(fmin(pid.output, pid.PWM_MAX), pid.PWM_MIN);

    return pid.output;
}

void pid_init(PID *pid, double kp, double ki, double kd, double kf, double target)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kf = kf; // 前馈系数
                  // pid->filter_alpha = fi;   // 输出滤波系数
    // pid->filtered_output = 0; // 初始化滤波状态

    pid->target = target;
    pid->integral = 0;
    pid->last_error = 0;
}

double pid_calculate(PID *pid, double feedback, double target)
{
    double error = target - feedback;

    // 动态死区（原死区0.5→动态范围0.1-3）
    float deadzone = fmax(0.1, fabs(target) * 0.07);
    if (fabs(error) < deadzone)
        error = 0;

    // 比例项
    double p_out = pid->kp * error;

    // 积分项（带抗饱和）
    const double integral_max = 80; // 50.0;
    pid->integral += error;
    if (pid->integral > integral_max)
        pid->integral = integral_max;
    else if (pid->integral < -integral_max)
        pid->integral = -integral_max;

    double i_out = pid->ki * pid->integral;

    // 微分项（计算误差变化率）
    double d_out = pid->kd * (error - pid->last_error);

    // // 前馈项
    // double f_out = pid->kf * target;
    // 自适应前馈（原前馈基础上增强）
    double f_out = pid->kf * target; //* (1 + 0.2*fabs(error/50.0));

    // 保存本次误差
    pid->last_error = error;

    // 计算输出
    double output = p_out + i_out + d_out + f_out;

    // 输出限幅
    const double output_max = 75;           //0719 提升速度上限为75
    const double output_min = 0;
    if (output >= 0)
        return fmax(fmin(fabs(output), output_max), output_min);
    else
        return -1 * fmax(fmin(fabs(output), output_max), output_min);
}



int get_state(const int current_state,
              const vector<Point> left_border, const vector<Point> right_border,
              const vector<Point> left_lost, const vector<Point> right_lost,
              vector<Point> &corners)
{
    static std::chrono::steady_clock::time_point last_time; // 上次出环时间
    single_corner_detect(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
    if (left_lost.size() > 10 && right_lost.size() > 10&&current_state==st_normal&&corners.size()>0) // 前方十字
    {
        last_time=std::chrono::steady_clock::now();         //记录开始进入十字的时间
            return st_cross;
    }

    if (current_state == st_normal || current_state == st_left_obstacle||current_state == st_right_obstacle) //  现在是直行   右圆环判定
    {
        corners.clear();            //清空角点
        single_corner_detect(left_border, right_border, left_lost, right_lost, corners, true);      // 右侧角点检测
        if (corners.size() > 0 && corners[0].y > DOWN_DETECT && right_lost.size() > 7 && left_lost.size() < 4&&is_border_continous(left_border,2.5)) // 有角点   并且前瞻点已经比角点高了        且右侧丢线左侧不丢线
        {
            is_right_circle=true;           //确认为右圆环
            last_time=std::chrono::steady_clock::now();         //记录开始进入圆环1的时间
            return st_Rcircle_1;
        }
        
        corners.clear();            //清空角点
        single_corner_detect(left_border, right_border, left_lost, right_lost, corners, false);      // 左侧角点检测
        if (corners.size() > 0 && corners[0].y > UP_DETECT+5 && left_lost.size() > 7 && right_lost.size() < 4&&is_border_continous(right_border,2.5)) // 有角点   并且前瞻点已经比角点高了        且右侧丢线左侧不丢线
        {
            is_right_circle=false;           //确认为左圆环
            last_time=std::chrono::steady_clock::now();         //记录开始进入圆环1的时间
            return st_Rcircle_1;
        }

    }

    if (current_state == st_Rcircle_1) // 圆环直行
    {
        //if ((left_lost.size() > 10 && is_right_circle&&corners.size()>0) || (right_lost.size() > 10 && !is_right_circle&&corners.size()>1)) // 纠正圆环
          //  return st_cross;
        const auto now_time=std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> time_1 = now_time-last_time;
        if(time_1.count()>1500)     //圆环1状态已经持续超过1.5s  
        return st_normal;           //退出圆环状态


        static bool last_gap_valid;
        if (is_right_circle)        //右圆环
        {
            corners.clear();            //清空角点
            single_corner_detect(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
            if (corners.size() >= 1)
            {
                if (corners[0].y < UP_DETECT && !is_valid_gap(right_border)&&last_gap_valid&&time_1.count()>300) // 前瞻点上方角点找到 并且右半边丢线<=10
                {
                    last_time=std::chrono::steady_clock::now();        
                    return st_Rcircle_2;
                }
                last_gap_valid=is_valid_gap(right_border);
            }
        }
        else
        {
            corners.clear();
            single_corner_detect(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
            if (corners.size() >= 1)
            {
                if (corners[0].y < UP_DETECT && !is_valid_gap(left_border)&&last_gap_valid&&time_1.count()>300) // 前瞻点上方角点找到 并且左半边丢线<=10
                {
                    last_time=std::chrono::steady_clock::now();        
                    return st_Rcircle_2;
                }
                last_gap_valid=is_valid_gap(left_border);
            }
        }
    }

    if (current_state == st_Rcircle_2) // 正在入环
    {
        corners.clear();
        single_corner_detect_no_vec(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
        const auto now_time=std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> time_2 = now_time-last_time;
        if (corners.size() == 0&&time_2.count()>300)       // 无角点  且距离入环状态已经过300ms
        {
            last_time=std::chrono::steady_clock::now();
            return st_Rcircle_3;                                                                                 // 环内
        }
    }

    if (current_state == st_Rcircle_3) // 已经入环了
    {
        corners.clear();
        single_corner_detect_no_dist(left_border, right_border, left_lost, right_lost, corners, !is_right_circle); // 左侧角点检测
        auto now_time=std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> time_3 = now_time-last_time;

        if (is_right_circle)
        {
            if (left_lost.size() >= 3&&time_3.count()>500&&(!is_valid_gap(left_border)||corners.size()>0))      //一般来说corners.size()>0这个条件更松
            {        
                last_time=std::chrono::steady_clock::now();
                    return st_Rcircle_4;        // 左侧丢线
            }
        }
        else
        {
            if (right_lost.size() >= 3&&time_3.count()>500&&(!is_valid_gap(right_border)||corners.size()>0))
            {
                last_time=std::chrono::steady_clock::now();
                    return st_Rcircle_4;
            }
        }
    }

    if (current_state == st_Rcircle_4) // 出圆环
    {
        corners.clear();
        single_corner_detect_no_vec(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
        auto now_time=std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> time_3 = now_time-last_time;

        if (is_right_circle)
        {
            if (left_lost.size() < 8&&time_3.count()>150)          //放宽出环调节
            {    
                return st_Rcircle_5;        //出环 直行
            }
        }
        else
        {
            if (right_lost.size() < 8&&time_3.count()>150)     
            {
                return st_Rcircle_5;        //出欢执行
            }
        }
    }

    if(current_state==st_Rcircle_5)
    {
        static int counter=0;
        corners.clear();
        single_corner_detect_no_vec(left_border, right_border, left_lost, right_lost, corners, is_right_circle); // 右侧角点检测
        if (corners.size() == 0)                                                                                 // 无角点
        {
            if(is_right_circle)
            {
                if(right_lost.size()<=5)
                counter++;
            }
            else
            { 
                if(left_lost.size()<=5)
                counter++;
            }
        }
        else counter=0;
        if(counter>=3)      //连续3帧没检测到角点且右边丢线少
        {
            counter=0;
            return st_normal;       //已经出环
        }
    }

    if(current_state==st_cross)
    {
        auto now_time=std::chrono::steady_clock::now();
        const std::chrono::duration<double, std::milli> time_3 = now_time-last_time;
        if(time_3.count()>700)          //十字状态超过700ms  结束十字
        {
            last_time=std::chrono::steady_clock::now();
            return st_normal;
        }
    }

    return current_state;
}



void cross_corner_detect(const vector<Point> left_border, const vector<Point> right_border,
                         const struct cross_lost lost_point,
                         Point &left_down_corner, Point &left_up_corner, Point &right_down_corner, Point &right_up_corner)
{
    const float dist_min_th = 2, dist_max_th = 4; // 判断角点与相邻点距离的阈值
    const int start_th_up = 8, start_th_down = 5; // 开始寻找角点的y坐标差
    const int sm_border_th = 55;                  // 假定的边界距离CAR_X的水平距离

    int i_left = 1, i_right = 1;

    if ((lost_point.L_y_max < UP_BORDER + 5 || lost_point.R_y_max < UP_BORDER + 5)) // 不太可能是十字里面   或者进入地太早了
    {
        cerr << "lost_point.L_y_max<UP_BORDER+5||lost_point.R_y_max<UP_BORDER+5" << endl;
        return;
    }

    if (lost_point.L_y_max < UP_DETECT || lost_point.R_y_max < UP_DETECT)
        return; // 前瞻点还没开始进入十字   不管

    //---------------左半边 下部分角点检测--------------------
    for (; i_left < left_border.size() - 1 && left_border[i_left].y > lost_point.L_y_max + start_th_down; i_left++)
        ; // 从靠近最低丢线点的位置下面6个像素往上找角点

    if (i_left > 5) // 说明左半边下部分还存在有效边界
    {
        // 角点与相邻点具有特征：1个离得近，另一个离得远
        for (; i_left < left_border.size() - 1; i_left++) // 从下往上  左边界找下方角点
        {
            // dist 两点距离
            const float dist1 = sqrt((left_border[i_left].x - left_border[i_left - 1].x) * (left_border[i_left].x - left_border[i_left - 1].x) + (left_border[i_left].y - left_border[i_left - 1].y) * (left_border[i_left].y - left_border[i_left - 1].y));
            const float dist2 = sqrt((left_border[i_left].x - left_border[i_left + 1].x) * (left_border[i_left].x - left_border[i_left + 1].x) + (left_border[i_left].y - left_border[i_left + 1].y) * (left_border[i_left].y - left_border[i_left + 1].y));
            if (fabs(dist1 - dist2) > 2.5)
            {
                // 不是边界点
                if (left_border[i_left].x > 2)
                    left_down_corner = left_border[i_left]; // 左下方角点
                // 否则应该是前一个点是角点
                else
                    left_down_corner = left_border[i_left - 1];
                break;
            }
        }
    }
    else
        left_down_corner = Point(CAR_X - sm_border_th, IMG_HEIGHT - 1); // 指定左下方固定位置为角点  便于补线

    //----------------左半边  上部分角点检测----------------
    i_left = left_border.size() - 2;
    for (; i_left >= 1 && left_border[i_left].y < lost_point.L_y_min - start_th_up; i_left--)
        ;

    for (; i_left >= 1; i_left--)
    {
        const float dist1 = sqrt((left_border[i_left].x - left_border[i_left - 1].x) * (left_border[i_left].x - left_border[i_left - 1].x) + (left_border[i_left].y - left_border[i_left - 1].y) * (left_border[i_left].y - left_border[i_left - 1].y));
        const float dist2 = sqrt((left_border[i_left].x - left_border[i_left + 1].x) * (left_border[i_left].x - left_border[i_left + 1].x) + (left_border[i_left].y - left_border[i_left + 1].y) * (left_border[i_left].y - left_border[i_left + 1].y));
        if (fabs(dist1 - dist2) > 2.5)
        {
            // 不是边界点
            if (left_border[i_left].x > 2)
                left_up_corner = left_border[i_left]; // 左上方角点
                                                      // 否则应该前一个点是角点
            else
                left_up_corner = left_border[i_left + 1];
            break;
        }
    }

    if ((left_down_corner.x < 0 && left_down_corner.y < 0) || (left_up_corner.x < 0 && left_up_corner.y < 0))
    {
        cerr << "L corner err" << endl;
        return;
    }

    //-----------右半边------------------
    for (; i_right < right_border.size() - 1 && right_border[i_right].y > lost_point.R_y_max + start_th_down; i_right++)
        ; // 从靠近最低丢线点的位置下面6个像素往上找角点

    if (i_right > 5) // 说明右半边下部分还存在有效边界
    {
        // 角点与相邻点具有特征：1个离得近，另一个离得远
        for (; i_right < right_border.size() - 1; i_right++) // 从下往上  左边界找下方角点
        {
            // dist 两点距离
            const float dist1 = sqrt((right_border[i_right].x - right_border[i_right - 1].x) * (right_border[i_right].x - right_border[i_right - 1].x) + (right_border[i_right].y - right_border[i_right - 1].y) * (right_border[i_right].y - right_border[i_right - 1].y));
            const float dist2 = sqrt((right_border[i_right].x - right_border[i_right + 1].x) * (right_border[i_right].x - right_border[i_right + 1].x) + (right_border[i_right].y - right_border[i_right + 1].y) * (right_border[i_right].y - right_border[i_right + 1].y));
            if (fabs(dist1 - dist2) > 2.5)
            {
                if (right_border[i_right].x < IMG_WIDTH - 2)
                    right_down_corner = right_border[i_right]; // 右下方角点
                else
                    right_down_corner = right_border[i_right - 1];
                break;
            }
        }
    }
    else
        right_down_corner = Point(CAR_X + sm_border_th, IMG_HEIGHT - 1); // 指定右下方固定位置为角点  便于补线

    //----------------右半边  上部分角点检测----------------
    i_right = right_border.size() - 2;
    for (; i_right >= 1 && right_border[i_right].y < lost_point.R_y_min - start_th_up; i_right--);

    for (; i_right >= 1; i_right--)
    {
        const float dist1 = sqrt((right_border[i_right].x - right_border[i_right - 1].x) * (right_border[i_right].x - right_border[i_right - 1].x) + (right_border[i_right].y - right_border[i_right - 1].y) * (right_border[i_right].y - right_border[i_right - 1].y));
        const float dist2 = sqrt((right_border[i_right].x - right_border[i_right + 1].x) * (right_border[i_right].x - right_border[i_right + 1].x) + (right_border[i_right].y - right_border[i_right + 1].y) * (right_border[i_right].y - right_border[i_right + 1].y));
        if (fabs(dist1 - dist2) > 2.5)
        {
            if (right_border[i_right].x < IMG_WIDTH - 2)
                right_up_corner = right_border[i_right]; // 左上方角点
            else
                right_up_corner = right_border[i_right + 1];
            break;
        }
    }

    if ((right_down_corner.x < 0 && right_down_corner.y < 0) || (right_up_corner.x < 0 && right_up_corner.y < 0))
    {
        cerr << "R corner err" << endl;
        return;
    }

    return;
}

void single_corner_detect(
    const vector<Point>& left_border,   // 修复1：改为引用传递，避免大对象拷贝导致的栈溢出
    const vector<Point>& right_border,  // 修复1：同上
    const vector<Point>& left_lost,     // 修复1：同上
    const vector<Point>& right_lost,    // 修复1：同上
    vector<Point>& corners, 
    const bool is_right
) 
{
    const int th = 6; 
    const int up_th = 80, down_th = 30;
    const int vector_gap = 2;

    // 修复2：提前检查容器大小，避免循环条件因无符号下溢导致越界
    auto check_size = [vector_gap](const vector<Point>& vec) {
        return vec.size() > static_cast<size_t>(2 * vector_gap); // 确保至少5个元素
    };

    if (is_right) {
        // 修复2：跳过不满足最小大小的容器
        if (!check_size(right_border)) return;

        for (int i_right = vector_gap; i_right < static_cast<int>(right_border.size()) - vector_gap; i_right++) {
            // 修复3：校验索引范围，防止 i_right±vector_gap 越界
            if (i_right - vector_gap < 0 || i_right + vector_gap >= static_cast<int>(right_border.size())) 
                continue;

             const float dist1 = sqrt((right_border[i_right].x - right_border[i_right - 1].x) * (right_border[i_right].x - right_border[i_right - 1].x) + (right_border[i_right].y - right_border[i_right - 1].y) * (right_border[i_right].y - right_border[i_right - 1].y));
            const float dist2 = sqrt((right_border[i_right].x - right_border[i_right + 1].x) * (right_border[i_right].x - right_border[i_right + 1].x) + (right_border[i_right].y - right_border[i_right + 1].y) * (right_border[i_right].y - right_border[i_right + 1].y));
            if (fabs(dist1 - dist2) > th) {
                if (right_border[i_right].x < IMG_WIDTH - 5 && right_border[i_right].y > UP_BORDER) {
                    const Point v1 = Point(right_border[i_right - vector_gap].x - right_border[i_right].x, 
                                         right_border[i_right - vector_gap].y - right_border[i_right].y);
                    const Point v2 = Point(right_border[i_right + vector_gap].x - right_border[i_right].x, 
                                         right_border[i_right + vector_gap].y - right_border[i_right].y);
                    const double angle = calcAngle_v(v1, v2);
                    if (angle >= down_th) {
                        corners.push_back(right_border[i_right]);
                    }
                }
            }
        }
    }
    else
    {
        // 修复2：跳过不满足最小大小的容器
        if (!check_size(left_border)) return;
        // 从下往上找
        for (int i_left = vector_gap; i_left < left_border.size() - vector_gap; i_left++)
        {
            //bugfix3 
            if (i_left - vector_gap < 0 || i_left + vector_gap >= static_cast<int>(left_border.size())) 
                continue;

            const float dist1 = sqrt((left_border[i_left].x - left_border[i_left - 1].x) * (left_border[i_left].x - left_border[i_left - 1].x) + (left_border[i_left].y - left_border[i_left - 1].y) * (left_border[i_left].y - left_border[i_left - 1].y));
            const float dist2 = sqrt((left_border[i_left].x - left_border[i_left + 1].x) * (left_border[i_left].x - left_border[i_left + 1].x) + (left_border[i_left].y - left_border[i_left + 1].y) * (left_border[i_left].y - left_border[i_left + 1].y));
            if (fabs(dist1 - dist2) > th) // 边界不连续
            {
                if (left_border[i_left].x > 5 && left_border[i_left].y > UP_BORDER) // 满足160x120约束
                {
                    const Point v1 = Point(left_border[i_left - vector_gap].x - left_border[i_left].x, left_border[i_left - vector_gap].y - left_border[i_left].y); // 前一个点到当前点 的向量
                    const Point v2 = Point(left_border[i_left + vector_gap].x - left_border[i_left].x, left_border[i_left + vector_gap].y - left_border[i_left].y); // 后一个点到当前点 的向量
                    const double angle = calcAngle_v(v1, v2);                                                                                                       // 计算角度
                    if (angle >= down_th)                                                                                                                           //  符合角度特征
                    {
                        corners.push_back(left_border[i_left]);
                    }
                }
            }
        }
    }

    return;
}

void single_corner_detect_no_vec(const vector<Point> left_border, const vector<Point> right_border,
                                 const vector<Point> left_lost, const vector<Point> right_lost,
                                 vector<Point> &corners, const bool is_right)
{
    const int th = 3; // 角点判断阈值(距离差)

    if (is_right) // 右侧
    {
        
        // 从上往下找
        for (int i_right = right_border.size() - 7; i_right >= 1; i_right--)
        {
            const float dist1 = sqrt((right_border[i_right].x - right_border[i_right - 1].x) * (right_border[i_right].x - right_border[i_right - 1].x) + (right_border[i_right].y - right_border[i_right - 1].y) * (right_border[i_right].y - right_border[i_right - 1].y));
            const float dist2 = sqrt((right_border[i_right].x - right_border[i_right + 1].x) * (right_border[i_right].x - right_border[i_right + 1].x) + (right_border[i_right].y - right_border[i_right + 1].y) * (right_border[i_right].y - right_border[i_right + 1].y));
            if (fabs(dist1 - dist2) > th) // 边界不连续
            {
                if (right_border[i_right].x < IMG_WIDTH - 5 && right_border[i_right].y > UP_BORDER) // 满足160x120约束
                {
                    corners.push_back(right_border[i_right]);
                }
            }
        }
    }
    else
    {
        // 从上往下找
        for (int i_left = left_border.size() - 7; i_left >= 1; i_left--)
        {
            const float dist1 = sqrt((left_border[i_left].x - left_border[i_left - 1].x) * (left_border[i_left].x - left_border[i_left - 1].x) + (left_border[i_left].y - left_border[i_left - 1].y) * (left_border[i_left].y - left_border[i_left - 1].y));
            const float dist2 = sqrt((left_border[i_left].x - left_border[i_left + 1].x) * (left_border[i_left].x - left_border[i_left + 1].x) + (left_border[i_left].y - left_border[i_left + 1].y) * (left_border[i_left].y - left_border[i_left + 1].y));
            if (fabs(dist1 - dist2) > th) // 边界不连续
            {
                if (left_border[i_left].x > 5 && left_border[i_left].y > UP_BORDER) // 满足160x120约束
                {
                    corners.push_back(left_border[i_left]);
                }
            }
        }
    }

    return;
}

void single_corner_detect_no_dist(const vector<Point> left_border, const vector<Point> right_border,
                                  const vector<Point> left_lost, const vector<Point> right_lost,
                                  vector<Point> &corners, const bool is_right)
{
    const int up_th = 90, down_th = 50;
    const int vector_gap = 4;

    // 修复2：提前检查容器大小，避免循环条件因无符号下溢导致越界
    auto check_size = [vector_gap](const vector<Point>& vec) {
        return vec.size() > static_cast<size_t>(2 * vector_gap); // 确保至少5个元素
    };

    if (is_right) // 右侧
    {
        // 修复2：跳过不满足最小大小的容器
        if (!check_size(right_border)) return;
        // 从下往上找
        for (int i_right = vector_gap; i_right < right_border.size() - vector_gap; i_right++)
        {
             // 修复3：校验索引范围，防止 i_right±vector_gap 越界
            if (i_right - vector_gap < 0 || i_right + vector_gap >= static_cast<int>(right_border.size())) 
                continue;
            if (right_border[i_right].x < IMG_WIDTH - 5 && right_border[i_right].y > UP_BORDER) // 满足160x120约束
            {
                const Point v1 = Point(right_border[i_right - vector_gap].x - right_border[i_right].x, right_border[i_right - vector_gap].y - right_border[i_right].y); // 前一个点到当前点 的向量
                const Point v2 = Point(right_border[i_right + vector_gap].x - right_border[i_right].x, right_border[i_right + vector_gap].y - right_border[i_right].y); // 后一个点到当前点 的向量
                const double angle = calcAngle_v(v1, v2);                                                                                                               // 计算角度
                if (angle >= down_th)                                                                                                                                   //  符合角度特征
                {
                    corners.push_back(right_border[i_right]);
                }
            }
        }
    }
    else
    {
        // 修复2：跳过不满足最小大小的容器
        if (!check_size(left_border)) return;
        // 从下往上找
        for (int i_left = vector_gap; i_left < left_border.size() - vector_gap; i_left++)
        {
             // 修复3：校验索引范围，防止 i_right±vector_gap 越界
            if (i_left - vector_gap < 0 || i_left + vector_gap >= static_cast<int>(left_border.size())) 
                continue;
            if (left_border[i_left].x > 5 && left_border[i_left].y > UP_BORDER) // 满足160x120约束
            {
                const Point v1 = Point(left_border[i_left - vector_gap].x - left_border[i_left].x, left_border[i_left - vector_gap].y - left_border[i_left].y); // 前一个点到当前点 的向量
                const Point v2 = Point(left_border[i_left + vector_gap].x - left_border[i_left].x, left_border[i_left + vector_gap].y - left_border[i_left].y); // 后一个点到当前点 的向量
                const double angle = calcAngle_v(v1, v2);                                                                                                       // 计算角度
                if (angle >= down_th)                                                                                                                           //  符合角度特征
                {
                    corners.push_back(left_border[i_left]);
                }
            }
        }
    }

    return;
}

double calcAngle_v(const Point v1, const Point v2)
{
    double dotProd = v1.x * v2.x + v1.y * v2.y;      // 向量内积
    double normV1 = sqrt(v1.x * v1.x + v1.y * v1.y); // 向量1的模
    double normV2 = sqrt(v2.x * v2.x + v2.y * v2.y); // 模

    if (normV1 == 0 || normV2 == 0)
    {
        cerr << "Norm=0!!!" << endl;
        return 180; // 避免除零错误
    }
    if (normV1 <= 5 && normV2 <= 5) // 避免曲线误判角点
        return 180;

    double cosTheta = dotProd / (normV1 * normV2);
    const float ang = acos(cosTheta) * 180.0 / CV_PI; // 转换为角度
    if (ang <= 90.0)
        return ang; // 返回锐角
    else
        return 180 - ang;
}





bool detect_st_obstacle(const Mat &edgeImage, std::vector<Point> &left_border, std::vector<Point> &right_border, const Mat &bin,
    struct cross_lost &lost_point,
    std::vector<Point> &left_valid_border, std::vector<Point> &right_valid_border)
{
    int prev_distance = -1;
    
    // 遍历已检测的边界点（从下往上）
    for (size_t i = 95; i <20; i--) { // 仅处理有效前瞻区域（y=5到115）
        //const int current_y = left_border[i].y;
        // 直接使用已计算的左右边界点
        const int left_x = left_border[i].x;
        const int right_x = right_border[i].x;
        const int current_distance = right_x - left_x;

        // 道宽突变检测
        if(prev_distance != -1 && abs(current_distance - prev_distance) > 5) {
            return true;
        }

        prev_distance = current_distance;
    }
    
    return false;
}


int dist_75(const vector<Point> corners)
{
    if(corners.size()<=1) return 160;       //只有0/1个角点,返回很大的值
    vector<int> dists;
    for (int i = 0; i < corners.size() - 1; i++)
    {
        dists.push_back(sqrt((corners[i + 1].y - corners[i].y)*(corners[i + 1].y - corners[i].y)+(corners[i + 1].x - corners[i].x)*(corners[i + 1].x - corners[i].x)));
    }
    sort(dists.begin(), dists.end());
    return dists[dists.size() * 0.75];
}





int correct_mid(const vector<Point> border,const bool is_right,vector<Point>& mid_c,const int gap)
{
    int c_gap = 0;
    mid_c.clear();

    if (is_debug)       //开启了视频调试
    {
        if (is_right) // 右边界有效,左边界丢线严重
        {
            for (int i = 0; i < border.size(); i++)
            {
                if (width_table[border[i].y] > 10) // 该y值在宽度表内能检索到有效值
                {
                    mid_c.push_back(Point(border[i].x - width_table[border[i].y] / 2, border[i].y)); // 根据宽度表修正中线
                }
                else
                {
                    return gap; // 宽度表查询不到 则取消修正 沿用原来的gap值
                }
            }
        }
        else // 左边界有效,右边界丢线严重
        {
            for (int i = 0; i < border.size(); i++)
            {
                if (width_table[border[i].y] > 10) // 该y值在宽度表内能检索到有效值
                {
                    mid_c.push_back(Point(border[i].x + width_table[border[i].y] / 2, border[i].y)); // 根据宽度表修正中线
                }
                else
                {
                    return gap; // 宽度表查询不到 则取消修正 沿用原来的gap值
                }
            }
        }
    }
    if (is_right)       //右边界有效,左边界丢线严重
    {
        for (int y = DOWN_DETECT; y >= UP_DETECT; y--)
        {
            int midX=0;
            for (int i = 0; i < border.size(); i++)
            {
                if (y == border[i].y)
                {
                    if(width_table[border[i].y] > 10)
                    midX =border[i].x-width_table[border[i].y]/2;
                    else
                    return gap;

                    c_gap+=CAR_X - midX;
                    //cout<<"midX:"<<midX<<"c_gap:"<<c_gap<<endl;
                    break;
                }
            }
        }
    }
    else        // 左边界有效,右边界丢线严重
    {
        for (int y = DOWN_DETECT; y >= UP_DETECT; y--)
        {
            int midX=0;
            for (int i = 0; i < border.size(); i++)
            {
                if (y == border[i].y)
                {
                    if(width_table[border[i].y] > 10)
                    midX =border[i].x+width_table[border[i].y]/2;
                    else
                    return gap;

                    c_gap+=CAR_X - midX;
                    //cout<<"midX:"<<midX<<"c_gap:"<<c_gap<<endl;
                    break;
                }
            }
        }

    }

    if(abs(c_gap)>abs(gap))     //选取一个更大的误差值返回
    return c_gap;
    else
    return gap;
}


void give_width()
{
    //bench_width Point .x 不是x坐标！ 不是x坐标！  是对应Point.y 的道宽！！
    const vector<Point> bench_width = {
        Point(117, 85),
        Point(115, 84),
        Point(113, 83),
        Point(111, 82),
        Point(109, 81),
        Point(107, 80),
        Point(105, 79),
        Point(103, 78),
        Point(101, 77),
        Point(98, 76),
        Point(95, 75),
        Point(93, 74),
        Point(91, 73),
        Point(89, 72),
        Point(87, 71),
        Point(85, 70),
        Point(83, 69),
        Point(81, 68),
        Point(79, 67),
        Point(77, 66),
        Point(75, 65),
        Point(72, 64),
        Point(70, 63),
        Point(67, 62),
        Point(65, 61),
        Point(63, 60),
        Point(61, 59),
        Point(59, 58),
        Point(57, 57),
        Point(55, 56),
        Point(53, 55),
        Point(50, 54),
        Point(48, 53),
        Point(46, 52),
        Point(44, 51),
        Point(41, 50),
        Point(39, 49),
        Point(37, 48),
        Point(35, 47),
        Point(32, 46),
        Point(30, 45),
        Point(28, 44),
        Point(26, 43),
        Point(24, 42),
        Point(21, 41),
        //20-40的宽度实际不可靠 仅为参考以免bug
        Point(20,40),
        Point(20,39),
        Point(19,38),
        Point(19,37),
        Point(18,36),
        Point(18,35),
        Point(17,34),
        Point(17,33),
        Point(16,32),
        Point(16,31),
        Point(15,30),
        Point(15,29),
        Point(14,28),
        Point(14,27),
        Point(13,26),
        Point(13,25),
        Point(12,24),
        Point(12,23),
        Point(11,22),
        Point(11,21),
        Point(10,20),
        Point(10,19),
        Point(9,18)
    };

    for(int i=0;i<bench_width.size();i++)
    {
        width_table[bench_width[i].y]=bench_width[i].x;
    }
    return;
}



bool is_border_continous(const vector<Point> border,const float threshold)
{
    for(int i=0;i<border.size()-1;i++)
    {
        const float dist=sqrt((border[i].x-border[i+1].x)*(border[i].x-border[i+1].x)+(border[i].y-border[i+1].y)*(border[i].y-border[i+1].y));
        if(dist>=threshold) return false;
    }
    return true;
}


bool is_valid_gap(const vector<Point> border)
{
    for(int i=0;i<border.size();i++)
    {
        if(border[i].y<=DOWN_DETECT&&border[i].y>=UP_DETECT)        //处在前瞻点高度范围内
        {
            if(border[i].x<=0||border[i].x>=159)
            return false;
        }
    }
    return true;
}
