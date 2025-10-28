#引入包含位姿传感器和电机驱动的库函数
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

from seekfree import IMU963RA
from seekfree import MOTOR_CONTROLLER
from seekfree import TSL1401
from seekfree import WIRELESS_UART

from display import *

import gc
import time
import math
import array

'''
初始化各个模块
'''

'''
无线串口初始化
'''

# 实例化 WIRELESS_UART 模块 参数是波特率
# 无线串口模块需要自行先配对好设置好参数
wireless = WIRELESS_UART(460800)

# 发送字符串的函数

# data_analysis 数据解析接口 适配逐飞助手的无线调参功能
data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
for i in range(0,8):
    # get_data 获取调参通道数据 只有一个参数范围 [0-7]
    data_wave[i] = wireless.get_data(i)

'''
拨码开关初始化
'''
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

'''
编码器初始化
'''
# 对应学习板的编码器接口 1/2
# 总共三个参数 两个必要参数一个可选参数 [pinA,pinB,invert]
# pinA - 编码器 A 相或 PLUS 引脚
# pinB - 编码器 B 相或 DIR 引脚
# invert - 可选参数 是否反向 可以通过这个参数调整编码器旋转方向数据极性
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3", False)

'''
CCD初始化
'''
# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(1)

'''
电机初始化
'''
# MOTOR_CONTROLLER 电机驱动模块 一共四个参数 [mode,freq,duty,invert]
# mode - 工作模式  [PWM_C24_DIR_C26,PWM_C25_DIR_C27]
# freq - PWM 频率
# duty - 可选参数 初始的占空比 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# invert - 可选参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

motor_duty_l = 0
motor_duty_r = 0
motor_duty_max = 9000

'''
位姿传感器初始化
'''
# 参数是采集周期 调用多少次 capture 更新一次数据,默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU963RA()


'''
函数与参数
'''
#矩阵计算函数
def matrix_add(A, B):
    """
    矩阵加法 A + B
    参数：
    A, B: 两个矩阵，使用嵌套列表表示，维度小于等于二维。
    返回：
    返回矩阵加法结果
    """
    # 确保A和B的维度相同
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("两个矩阵的维度必须相同")

    # 进行逐元素加法
    return [
        [A[i][j] + B[i][j] for j in range(len(A[0]))]  # 遍历每一行，每行的每个元素
        for i in range(len(A))  # 遍历每一行
    ]

def matrix_sub(A, B):
    """
    任意尺寸矩阵减法 A - B
    参数：
    A, B: 两个矩阵，使用嵌套列表表示，维度小于等于二维。
    返回：
    返回矩阵减法结果
    """
    # 确保A和B的维度相同
    if len(A) != len(B) or len(A[0]) != len(B[0]):
        raise ValueError("两个矩阵的维度必须相同")

    # 进行逐元素减法
    return [
        [A[i][j] - B[i][j] for j in range(len(A[0]))]  # 遍历每一行，每行的每个元素
        for i in range(len(A))  # 遍历每一行
    ]


def matrix_scale(A, scalar):
    """
    2x2矩阵数乘 A * scalar
    参数：
    A: 2x2 矩阵
    scalar: 标量
    返回：
    返回数乘后的矩阵
    """
    return [
        [A[0][0] * scalar, A[0][1] * scalar],
        [A[1][0] * scalar, A[1][1] * scalar]
    ]

def matrix_multiply(A, B):
    """
    执行矩阵乘法 A * B。
    参数：
    A, B: 任意维度的矩阵，假设 A 的列数等于 B 的行数。
    
    返回：
    返回 A * B 的结果，仍然是一个矩阵。
    """
    # 获取 A 和 B 的行列数
    rows_A = len(A)
    cols_A = len(A[0])
    rows_B = len(B)
    cols_B = len(B[0])
    
    # 确保 A 的列数与 B 的行数相同
    if cols_A != rows_B:
        raise ValueError("矩阵 A 的列数必须等于矩阵 B 的行数")
    
    # 创建一个结果矩阵，大小为 A 的行数 x B 的列数
    result = [[0] * cols_B for _ in range(rows_A)]
    
    # 计算矩阵乘法
    for i in range(rows_A):
        for j in range(cols_B):
            for k in range(cols_A):  # 或者 k in range(rows_B)，因为 cols_A == rows_B
                result[i][j] += A[i][k] * B[k][j]
    
    return result


def matrix_transpose(A):
    """
    2x2矩阵转置 A^T
    参数：
    A: 2x2 矩阵
    返回：
    返回矩阵 A 的转置
    """
    return [
        [A[0][0], A[1][0]],
        [A[0][1], A[1][1]]
    ]

def matrix_inverse(A):
    """
    求 2x2 矩阵 A 的逆
    参数：
    A: 2x2 矩阵
    返回：
    返回 A 的逆矩阵
    """
    det = A[0][0] * A[1][1] - A[0][1] * A[1][0]
    
    if det == 0:
        raise ValueError("矩阵不可逆，行列式为0")
    
    inv_det = 1.0 / det  # 计算行列式的倒数
    return [
        [A[1][1] * inv_det, -A[0][1] * inv_det],
        [-A[1][0] * inv_det, A[0][0] * inv_det]
    ]

def identity_matrix():
    """
    返回一个 2x2 单位矩阵
    返回：
    返回 2x2 单位矩阵 [[1, 0], [0, 1]]
    """
    return [[1, 0], [0, 1]]

def covariance_matrix(X, Y):
    """
    计算 X 和 Y 之间的协方差矩阵。
    X 和 Y 是一组样本数据。
    参数：
    X: 一个包含 n 个元素的列表，表示 X 变量的样本数据。
    Y: 一个包含 n 个元素的列表，表示 Y 变量的样本数据。
    
    返回：
    返回一个 2x2 协方差矩阵，形式如下：
    [[Var(X), Cov(X,Y)],
     [Cov(X,Y), Var(Y)]]
    """
    # 样本数
    N = len(X)
    
    # 计算均值
    mean_X = sum(X) / N
    mean_Y = sum(Y) / N
    
    # 计算协方差和方差
    cov_XY = sum((X[i] - mean_X) * (Y[i] - mean_Y) for i in range(N)) / N
    var_X = sum((X[i] - mean_X) ** 2 for i in range(N)) / N
    var_Y = sum((Y[i] - mean_Y) ** 2 for i in range(N)) / N
    
    # 返回 2x2 协方差矩阵
    return [
        [var_X, cov_XY],
        [cov_XY, var_Y]
    ]

#卡尔曼滤波

#卡尔曼滤波参数
x = [[0],[0]]
p = [[550,0],[0,550]]
F = [[1,0.001],[0,1]]
H = [[1,0],[0,1]]
Q = [[1e-4,0],[0,1e-3]]
R = [[0.01,0],[0,0.25]]

#卡尔曼滤波函数
def kalman_filter(x, p, z, F, H, Q, R):
    """
    卡尔曼滤波器的实现
    参数：
    x - 上一次的状态估计值 (1xN 向量)
    p - 上一次的协方差矩阵 (NxN 矩阵)
    z - 本次的测量值 (1xN 向量)
    F - 状态转移矩阵 (NxN 矩阵)
    H - 观测矩阵 (MxN 矩阵)
    Q - 过程噪声协方差矩阵 (NxN 矩阵)
    R - 观测噪声协方差矩阵 (MxM 矩阵)
    
    返回：
    x_est - 更新后的状态估计值
    p_est - 更新后的协方差矩阵
    """
    # 预测步骤
    # 预测新的状态估计
    
    x_pred = matrix_multiply(F, x)  # x_pred = F * x
    # 预测新的协方差
    p_pred = matrix_multiply(F, matrix_multiply(p, matrix_transpose(F)))  # p_pred = F * P * F^T
    p_pred = matrix_add(p_pred, Q)  # p_pred = p_pred + Q

    # 更新步骤
    # 计算卡尔曼增益
    H_p_pred = matrix_multiply(H, p_pred)  # H * p_pred
    H_p_pred_HT = matrix_multiply(H_p_pred, matrix_transpose(H))  # H * p_pred * H^T
    S = matrix_add(H_p_pred_HT, R)  # S = H * p_pred * H^T + R
    S_inv = matrix_inverse(S)  # S_inv = (S)^-1
    K = matrix_multiply(p_pred, matrix_multiply(matrix_transpose(H), S_inv))  # K = p_pred * H^T * S_inv

    # 更新状态估计
    y = matrix_sub(z, matrix_multiply(H, x_pred))  # y = z - H * x_pred
    x_est = matrix_add(x_pred, matrix_multiply(K, y))  # x_est = x_pred + K * y

    # 更新协方差矩阵
    I = identity_matrix()  # 单位矩阵
    KH = matrix_multiply(K, H)  # K * H
    p_est = matrix_sub(p_pred, matrix_multiply(KH, p_pred))  # p_est = p_pred - K * H * p_pred

    return x_est, p_est

#CCD函数

#待定数据
straight_road_width = 50
total_road_width_f = 148
total_road_width_n = 148

import array

# 卷积操作
def convolve_1d(data, kernel):
    """对一维数据进行卷积操作。"""
    kernel_size = len(kernel)
    padding = kernel_size // 2
    result = [0] * len(data)

    for i in range(len(data)):
        sum_val = 0
        for j in range(kernel_size):
            idx = i + j - padding
            if 0 <= idx < len(data):
                sum_val += data[idx] * kernel[j]
        
        result[i] = int(round(sum_val))

    return result

# 01化处理
def threshold_and_denoise(data):
    threshold = sum(data) / len(data)
    
    binary_data = [100 if value > threshold else 1 for value in data]
    
    return binary_data

# 主函数，处理 CCD 数据
def process_ccd_data(ccd_data):
    data = array.array('h', ccd_data)

    kernel = [1, 1, 1]  # 卷积核
    kernel = [k / sum(kernel) for k in kernel]  # 归一化卷积核

    # 卷积操作
    convolved_data = convolve_1d(data, kernel)
    #归一化操作
    denoised_data = threshold_and_denoise(convolved_data)

    return denoised_data


#道路提取
def ccd_road_find(ccd_data):
    '''
    提取道路，计算道路宽度
    '''
    #提取道路中点
    n = len(ccd_data)
    mid_point = n // 2
    
    line = 0
    left = mid_point
    right = mid_point
    
    #中点向左右扫描交界点
    while left >= 0 or right < n:

        if ccd_data[left] == 0:
            line_pos = left
            break
        
        if ccd_data[right] == 100:
            line_pos = right
            break

        # 向左右扫描
        left = left - 1
        right = right + 1
    
    # 计算两个交界点的距离
    return line

#pid

#pid参数
pid_out = 0
pid_out_l = 0
pid_out_r = 0

#转向参数
line_pos = 0
car_pos = 0

kp_dis = 0
kd_dis = 0
in_dis_last = 0

#速度参数
#范围 ±10000
pur_v = 0

inte_v_limit = 500

kp_v_l = 84 * 0.00001
ki_v_l = 21 * 0.0000001
integral_v_l = 0

kp_v_r = kp_v_l
ki_v_r = ki_v_l
integral_v_r = 0


#直立参数
#前倾为正，弧度制
pur_theita = 2.254

kp_theita_l = 0.742
kd_theita_l = 1.062
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 5000

kp_w_y_l = 13558
ki_w_y_l = 700
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
integral_w_y_r = 0

#pid函数
def v_pid(pur, data, kp, ki, inte, v_limit):
    
    in_now =  pur - data
    integral = inte + in_now
    
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out 


def theita_pid(pur, data, kp, kd, in_last):

    in_now = data - pur
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out


def w_y_pid(pur, data, kp, ki, inte ,w_y_limit):
    in_now = pur - data
    integral = inte + in_now
    
    if integral > w_y_limit:
        integral = w_y_limit
    elif integral < -w_y_limit:
        integral = -w_y_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out

def dis_pid(pur, data, kp, kd, in_last):
    in_now = pur - data
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out

def follow_road(line_pos,v_l, v_r, theita_real, w_y_real):
    '''
    pid车辆循迹
    '''
    global in_dis_last
    global integral_v_l, integral_v_r
    global in_theita_last_l, in_theita_last_r, integral_w_y_l, integral_w_y_r
    '''转向环'''

    in_dis_last, pur_w_z = dis_pid(line_pos, 0, kp_dis, kd_dis, in_dis_last)

    '''前进环'''
    v_turn = pur_w_z
    pur_v_l = pur_v + v_turn
    pur_v_r = pur_v - v_turn
    
    integral_v_l, pid_out_l = v_pid(pur_v_l, v_l, kp_v_l, ki_v_l, integral_v_l, inte_v_limit)
    integral_v_r, pid_out_r = v_pid(pur_v_r, v_r, kp_v_r, ki_v_r, integral_v_r, inte_v_limit)
    
    pur_theita_l = pid_out_l + pur_theita
    pur_theita_r = pid_out_r + pur_theita
    
    #pur_theita_l = pid_out_l + theita_real
    #pur_theita_r = pid_out_r + theita_real
    
    '''
    直立环
    '''
    #pur_theita_l = pur_theita
    #pur_theita_r = pur_theita

    in_theita_last_l, pid_out_l = theita_pid(pur_theita_l, theita_real, kp_theita_l, kd_theita_l, in_theita_last_l)
    in_theita_last_r, pid_out_r = theita_pid(pur_theita_r, theita_real, kp_theita_r, kd_theita_r, in_theita_last_r)
    
    pur_w_y_l = pid_out_l
    pur_w_y_r = pid_out_r
    integral_w_y_l, pid_out_l = w_y_pid(pur_w_y_l, w_y_real, kp_w_y_l, ki_w_y_l, integral_w_y_l ,inte_w_y_limit)
    integral_w_y_r, pid_out_r = w_y_pid(pur_w_y_r, w_y_real, kp_w_y_r, ki_w_y_r, integral_w_y_r ,inte_w_y_limit)
    
    motor_duty_l = pid_out_l
    motor_duty_r = pid_out_r
    
    if(motor_duty_l >= motor_duty_max):
        motor_duty_l = motor_duty_max
    elif(motor_duty_l <= -motor_duty_max):
        motor_duty_l = -motor_duty_max
    
    if(motor_duty_r >= motor_duty_max):
        motor_duty_r = motor_duty_max
    elif(motor_duty_r <= -motor_duty_max):
        motor_duty_r = -motor_duty_max
    
    #更新电机PWM输出
    #wireless.send_oscilloscope(pid_out_l,pid_out_r, pur_theita_l,theita_real,pur_w_y_l,w_y_real)
    wireless.send_oscilloscope(motor_duty_l,motor_duty_r,v_l,v_r, pur_theita,pur_v,line_pos)
    motor_l.duty(motor_duty_l)
    motor_r.duty(motor_duty_r)

'''
定时器初始化（含有tof和ccd）
'''
ticker_flag = False
ticker_count = 0

def time_pit_handler_1(time):
    global v_l,v_r , tof_data
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)
    
    if (ticker_flag and ticker_count % 5 == 0):
        v_l = encoder_l.get() 
        v_r = encoder_r.get()
        tof_data = tof.get()
   
pit = ticker(1)
# 关联采集接口
pit.capture_list(encoder_l, encoder_r,imu , ccd )
# 关联 Python 回调函数
pit.callback(time_pit_handler_1)
# 每1ms触发一次回调函数
pit.start(1)

#函数
while True:   
    if (ticker_flag and ticker_count % 8 == 0):
        '''
        位姿解算和卡尔曼滤波
        '''
        #位姿传感器读数
        imu_data = imu.get()
        
        #位姿解算
        if(imu_data[1] == 0):
            a = 0.001
        else:
            a = imu_data[1]
        theita = math.atan2(imu_data[2] , a)
        
        w_y_dps = imu_data[3]
        
        #单位换算
        trans = 250 / 65535 * 2**1 * math.pi / 180
        w_y = w_y_dps * trans
        
        if theita <= 0:
            theita = 2 * math.pi + theita
        
        #卡尔曼滤波
        z = [[theita] ,[w_y]]
        x, p = kalman_filter(x, p, z, F, H, Q, R)
        
        theita_cal =  x[0][0]
        w_y_cal = x[1][0] - 0.000327907906991503
        
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data1 = ccd.get(0)
        modified_list1 = ccd_data1[11:-11]
        #ccd_data2 = ccd.get(1)
        #modified_list2 = ccd_data2[11:-11]
        
        road_data1 = process_ccd_data(modified_list1)
        #road_data2 = process_ccd_data(modified_list2)
        
        line_pos = ccd_road_find(road_data1)
        
        ticker_flag = False
        
        
    elif (ticker_flag and ticker_count % 10 == 0):
        follow_road(line_pos, v_l, v_r, theita_cal, w_y_cal)
        ticker_flag = False
    
    '''
    调参用无线串口
    '''
    #定期进行数据解析
    data_flag = wireless.data_analysis()
    for i in range(0,8):
        # 判断哪个通道有数据更新
        if (data_flag[i]):
            # 数据更新到缓冲
            data_wave[i] = wireless.get_data(i)
            if i == 2:
                integral_w_y_l = 0
                integral_w_y_r = 0
                integral_v_l = 0
                integral_v_r = 0
            elif i == 3:
                integral_w_y_l = 0
                integral_w_y_r = 0
                integral_v_l = 0
                integral_v_r = 0
            # 将更新的通道数据输出到 Thonny 的控制台
            print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i,data_wave[i]))
    
    if data_wave[0] != 0:
        kp_dis = data_wave[0]
    
    if data_wave[1] != 0:
        kd_dis = data_wave[1]
    
    if data_wave[2] != 0:
        pur_theita = data_wave[2]
        
    if data_wave[3] != 0:
        pur_v = data_wave[3]
    
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()



