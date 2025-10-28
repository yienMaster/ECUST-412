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


#无线串口初始化
wireless = WIRELESS_UART(460800)

data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
for i in range(0,8):
    # get_data 获取调参通道数据 只有一个参数范围 [0-7]
    data_wave[i] = wireless.get_data(i)

#编码器初始化
encoder_l = encoder("D0", "D1", False)
encoder_r = encoder("D2", "D3", True)

#CCD初始化
ccd = TSL1401(2)


#电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

motor_duty_l = 0
motor_duty_r = 0
motor_duty_max = 7500

#位姿传感器初始化
imu = IMU963RA()


'''
函数与参数
'''
#卡尔曼滤波
def kalman_filter_explicit(x, p, z, F, H, Q, R):
    """
    直接计算卡尔曼滤波，优化计算效率
    x: [x1, x2]  当前状态估计
    p: [[p11, p12], [p21, p22]]  当前误差协方差
    z: [z1, z2]  测量值
    F, H, Q, R:  卡尔曼滤波参数矩阵
    """

    # 提前计算 F 的平方和乘积，减少重复运算
    F00_2 = F[0][0] * F[0][0]
    F01_2 = F[0][1] * F[0][1]
    F10_2 = F[1][0] * F[1][0]
    F11_2 = F[1][1] * F[1][1]
    F00_F01 = 2 * F[0][0] * F[0][1]
    F10_F11 = 2 * F[1][0] * F[1][1]

    # **预测步骤**
    x1_pred = F[0][0] * x[0] + F[0][1] * x[1]
    x2_pred = F[1][0] * x[0] + F[1][1] * x[1]

    # **预测 P_pred**
    p11_pred = F00_2 * p[0][0] + F00_F01 * p[1][0] + F01_2 * p[1][1] + Q[0][0]
    p12_pred = F[0][0] * F[1][0] * p[0][0] + (F[0][0] * F[1][1] + F[0][1] * F[1][0]) * p[1][0] + F[0][1] * F[1][1] * p[1][1] + Q[0][1]
    p21_pred = p12_pred  # 由于 P 是对称矩阵，p12_pred 和 p21_pred 相同
    p22_pred = F10_2 * p[0][0] + F10_F11 * p[1][0] + F11_2 * p[1][1] + Q[1][1]

    # **计算 S = H * P_pred * H^T + R**
    S11 = p11_pred + R[0][0]
    S12 = p12_pred + R[0][1]
    S21 = S12  # 对称矩阵
    S22 = p22_pred + R[1][1]

    # **计算 S 的逆**
    det_S = S11 * S22 - S12 * S21
    inv_det_S = 1 / det_S  # 直接计算倒数，减少除法操作
    S11_inv = S22 * inv_det_S
    S12_inv = -S12 * inv_det_S
    S21_inv = S12_inv  # 对称矩阵
    S22_inv = S11 * inv_det_S

    # **计算卡尔曼增益 K**
    K11 = p11_pred * S11_inv + p12_pred * S21_inv
    K12 = p11_pred * S12_inv + p12_pred * S22_inv
    K21 = p21_pred * S11_inv + p22_pred * S21_inv
    K22 = p21_pred * S12_inv + p22_pred * S22_inv

    # **计算测量残差 y = z - H * x_pred**
    y1 = z[0] - x1_pred
    y2 = z[1] - x2_pred

    # **更新状态估计**
    x1_est = x1_pred + K11 * y1 + K12 * y2
    x2_est = x2_pred + K21 * y1 + K22 * y2

    # **更新协方差 P_est**
    P11_est = (1 - K11) * p11_pred - K12 * p21_pred
    P12_est = (1 - K11) * p12_pred - K12 * p22_pred
    P21_est = -K21 * p11_pred + (1 - K22) * p21_pred
    P22_est = -K21 * p12_pred + (1 - K22) * p22_pred

    return [x1_est, x2_est], [[P11_est, P12_est], [P21_est, P22_est]]


# **初始化参数**
x = [0, 0]
p = [[550, 0], [0, 550]]

x_o = [0, 0]
p_o = [[550, 0], [0, 550]]

# 状态转移矩阵 F
F = [[1, 0.001], [0, 1]]
F_o = [[1, 0.01], [0, 1]]

# 观测矩阵 H
H = [[1, 0], [0, 1]]

# 过程噪声协方差 Q
Q = [[0.01, 0], [0, 0.01]]
Q_o = [[0.01, 0], [0, 0.01]]

# 观测噪声协方差 R
R = [[0.09, 0], [0, 0.09]]
R_o = [[0.07, 0], [0, 0.07]]

#CCD函数

#待定数据
left_edge = -53
right_edge = 53
ccd_limit = 50

def find_road(data , width):
    """
    差比和滤波处理CCD数据，并同时寻找道路边线
    """
    global left_edge , right_edge , ccd_limit
    
    n = len(data)
    filtered_data = array.array('h', [0] * n)
    pos_values = []  # 正值数组
    neg_values = []  # 负值数组
    
    for i in range(12, n - 11):
        left = data[i - 1]
        center = data[i]
        right = data[i + 1]
        
        # 计算差比和
        diff_ratio = (left - right) / (center + 1) * 255  # 避免除零
        filtered_data[i] = abs(int(diff_ratio))  # 归一化到 0-255 范围
        
        # 记录大于100的点，并存储位置相对于64的偏移
        if abs(diff_ratio) > ccd_limit:
            offset = i - 64
            if diff_ratio > 0 and offset > 0:
                pos_values.append(offset)
            if diff_ratio < 0 and offset < 0:
                neg_values.append(offset)
    
    # 默认边界值
    left_edge, right_edge = -53, 53
    
    if neg_values:
        left_edge = min(neg_values, key=abs)
    if pos_values:
        right_edge = min(pos_values, key=abs)
    
    if left_edge > -53 and right_edge < 53:
        left_edge = left_edge
        right_edge = right_edge
    if left_edge < -53 and right_edge < 53 :
        left_edge = right_edge - width
    if left_edge > -53 and right_edge > 53 :
        right_edge = left_edge + width
    if left_edge < -53 and right_edge > 53 :
        left_edge, right_edge = -53, 53
    
    road_width = right_edge - left_edge if right_edge > left_edge else 0
    return left_edge , right_edge , road_width

#pid

#pid参数
pid_out = 0
pid_out_l = 0
pid_out_r = 0

#转向参数
v_turn = 0
line_pos = 0
car_pos = 0


in_dis_last = 0

kp_1 = 0
kp_2 = 0
kd_1 = 0
kd_2 = 0

'''
kp_dis = 0
kd_dis = 0
in_dis_last = 0

pur_w_z = 0

kp_w_z = 0
ki_w_z = 0
integral_w_z = 0
inte_w_z_limit = 25
'''
#速度参数
#范围 ±10000
pur_v = 0

inte_v_limit = 100

kp_v_l = 331 * 0.00001
ki_v_l = 194 * 0.0000001
kd_v_l = 404 * 0.00001

kp_v_r = kp_v_l 
ki_v_r = ki_v_l
kd_v_r = kd_v_l

in_v_last_l = 0
in_v_last_r = 0
integral_v_r = 0
integral_v_l = 0

#直立参数
#前倾为正，弧度制
pur_theita = 2.2438

kp_theita_l = 0.57
kd_theita_l = 1.092
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 25

kp_w_y_l = 17900
ki_w_y_l = 920
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
integral_w_y_r = 0

#pid函数
def v_pid(pur, data, kp, ki, kd, inte,  in_last , v_limit):

    in_now =  pur - data
    integral = inte + in_now
    
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral +  kd * (in_now - in_last)
    
    return integral, in_now, out


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

'''
def dis_pid(pur, data, kp, kd, in_last):

    in_now = data - pur
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out

def w_z_pid(pur, data, kp, ki, inte ,w_z_limit):

    in_now = data - pur
    integral = inte + in_now
    
    if integral > w_z_limit:
        integral = w_z_limit
    elif integral < -w_z_limit:
        integral = -w_z_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out
'''

def follow_line(pur, data, gyro , kp1, kd1, kp2, kd2, in_last):

    in_now = pur - data
        
    out = kp1 * in_now + abs( in_now ) * in_now * kp2 + (in_now - in_last) * kd1 + gyro * kd2
    
    in_last = in_now
    
    return in_last, out


def run( v_turn ,v_l, v_r, theita_real, w_y_real):
    '''
    直立环
    '''
    global integral_v_l, integral_v_r , in_v_last_l, in_v_last_r
    global in_theita_last_l, in_theita_last_r, integral_w_y_l, integral_w_y_r
    
    '''前进环'''
    pur_v_l = pur_v - v_turn
    pur_v_r = pur_v + v_turn
    #pur_v_l = pur_v
    #pur_v_r = pur_v
    
    integral_v_l, in_v_last_l, pid_out_l = v_pid(pur_v_l, v_l, kp_v_l, ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
    integral_v_r, in_v_last_r, pid_out_r = v_pid(pur_v_r, v_r, kp_v_r, ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
    
    pur_theita_l = pur_theita + pid_out_l
    pur_theita_r = pur_theita + pid_out_r
    #pur_theita_l = pur_theita
    #pur_theita_r = pur_theita

    in_theita_last_l, pid_out_l = theita_pid(pur_theita_l, theita_real, kp_theita_l, kd_theita_l, in_theita_last_l)
    in_theita_last_r, pid_out_r = theita_pid(pur_theita_r, theita_real, kp_theita_r, kd_theita_r, in_theita_last_r)
    
    pur_w_y_l = -pid_out_l
    pur_w_y_r = -pid_out_r
    
    integral_w_y_l, pid_out_l = w_y_pid(pur_w_y_l, w_y_real, kp_w_y_l, ki_w_y_l, integral_w_y_l ,inte_w_y_limit)
    integral_w_y_r, pid_out_r = w_y_pid(pur_w_y_r, w_y_real, kp_w_y_r, ki_w_y_r, integral_w_y_r ,inte_w_y_limit)
    
    motor_duty_l = -pid_out_l
    motor_duty_r = -pid_out_r
    
    if(motor_duty_l >= motor_duty_max):
        motor_duty_l = motor_duty_max
    elif(motor_duty_l <= -motor_duty_max):
        motor_duty_l = -motor_duty_max
    
    if(motor_duty_r >= motor_duty_max):
        motor_duty_r = motor_duty_max
    elif(motor_duty_r <= -motor_duty_max):
        motor_duty_r = -motor_duty_max
    
    #更新电机PWM输出
    #wireless.send_oscilloscope(motor_duty_l,kp_theita_l,kd_theita_l,kp_w_y_l,ki_w_y_l,pur_theita,theita_real,integral_w_y_l)
    #wireless.send_oscilloscope(motor_duty_l,pur_theita_l,theita_real, pur_w_y_l, w_y_real)
    #wireless.send_oscilloscope(v_l , integral_v_l , pur_v , pur_theita , kp_v_l * 100000 , ki_v_l * 10000000 , kd_v_l * 100000)
    
    motor_l.duty(motor_duty_l)
    motor_r.duty(motor_duty_r)
    
def follow_road(line_pos,  w_z_real , v_l, v_r, theita_real, w_y_real):
    '''
    pid车辆循迹
    '''
    #global in_dis_last, integral_w_z, v_turn
    
    global in_dis_last, v_turn
    '''转向环'''
    
    '''
    in_dis_last, pur_w_z = dis_pid(line_pos, 0, kp_dis, kd_dis, in_dis_last)
    integral_w_z, v_turn = w_z_pid(pur_w_z, w_z_real, kp_w_z, ki_w_z, integral_w_z ,inte_w_z_limit)
    '''
    in_dis_last, v_turn = follow_line(line_pos, 0, w_z_real , kp_1, kd_1, kp_2, kd_2, in_dis_last)
    #wireless.send_oscilloscope(v_turn, kp_dis, kd_dis, kp_w_z, ki_w_z, line_pos, integral_w_z)
    wireless.send_oscilloscope(pur_theita , kp_1, kd_1, kp_2, kd_2, line_pos , left_edge , right_edge)
    
    run( v_turn ,v_l, v_r, theita_real, w_y_real)

'''
定时器初始化
'''

cal_flag = True
ticker_count = 1
theita_cal = 0
w_y_cal = 0
v_l = 0
v_r = 0
omiga_cal = 0
w_z_cal = 0
run_flag = 0
imu_data = imu.get()

def time_pit_handler_1(time):
    global v_l, v_r ,theita_cal, w_y_cal, line_pos
    global cal_flag, x, p
    global ticker_count, w_z_cal
    ticker_count = (ticker_count + 1) if (ticker_count < 2) else (1)
    
    '''
    位姿解算和卡尔曼滤波
    '''
    
    #位姿传感器读数
    v_r = encoder_l.get()
    v_l = encoder_r.get()
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
    w_y = - w_y_dps * trans
    
    if theita <= 0:
        theita = 2 * math.pi + theita
    
    #卡尔曼滤波
    z = [theita,w_y]
    x, p = kalman_filter_explicit(x, p, z, F, H, Q, R)
    
    theita_cal =  x[0]
    w_y_cal = x[1] + 0.000327907906991503
    
    #wireless.send_oscilloscope(w_y_cal, w_y, theita_cal, theita)
    
    if ticker_count == 1 and run_flag:
        run(v_turn ,v_l, v_r, theita_cal, w_y_cal)
    elif ticker_count == 2 and run_flag:
        cal_flag = True
        
        #print(line_pos, w_z_cal)
        follow_road(line_pos, w_z_cal , v_l, v_r, theita_cal, w_y_cal)
pit = ticker(1)
# 关联采集接口
pit.capture_list(encoder_l, encoder_r, ccd, imu)
# 关联 Python 回调函数
pit.callback(time_pit_handler_1)
# 每5ms触发一次回调函数
pit.start(5)


#函数
while True:
    #读取传感器   
    if cal_flag :
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        
        #75 60
        left_n, right_n, width_n = find_road(ccd_data1 , 75)
        #print(left_n, right_n)
        left_f, right_f, width_f = find_road(ccd_data2 , 60)
        #wireless.send_oscilloscope(width_n, width_f)
        line_pos = (0.4 * (left_n + right_n) + 0.6 * (left_f + right_f))*(1/2)        
        #位姿解算
        mx = imu_data[6] + 3216
        my = imu_data[7] - 24 
        mz = imu_data[8] + 1181
        
        w = (1/2) * math.pi - theita_cal
        
        #偏航角
        mx_g = my * math.cos(w) - mz * math.sin(w)
        
        if(mx == 0):
            my_g = 0.0001
        else:
            my_g = mx 
        omiga = math.atan2(mx_g , my_g)
        
        w_z_dps = imu_data[4] * math.sin(theita_cal) + imu_data[5] * math.cos(theita_cal)
        
        #角速度换算
        trans = 250 / 65535 * 2**1 * math.pi / 180
        w_z = w_z_dps * trans
        
        #卡尔曼滤波
        z_o = [omiga, w_z]
        x_o, p_o = kalman_filter_explicit(x_o, p_o, z_o, F_o, H, Q_o, R_o)
        
        omiga_cal =  x_o[0]
        w_z_cal = x_o[1] + 0.001
        
        #wireless.send_oscilloscope(w_z_cal, w_z)
        
        cal_flag = False
        
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
            if i == 7 or i == 4:
                integral_w_y_l = 0
                integral_w_y_r = 0
                integral_v_l = 0
                integral_v_r = 0
                integral_w_z = 0
            # 将更新的通道数据输出到 Thonny 的控制台
            print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i,data_wave[i]))
            
    '''
    kp_theita_l = data_wave[0]
    kp_theita_r = kp_theita_l
    
    kd_theita_l = data_wave[1]
    kd_theita_r = kd_theita_l
    
    kp_w_y_l = data_wave[2]
    kp_w_y_r = kp_w_y_l
    
    ki_w_y_l = data_wave[3]
    ki_w_y_r = ki_w_y_l
    
    if data_wave[4] != 0:
        pur_theita = data_wave[4]
    '''
    '''
    pur_v = data_wave[4]
    
    if data_wave[0] != 0:
        kp_v_l = data_wave[0] * 0.00001
        kp_v_r = kp_v_l
             
    if data_wave[1] != 0:
        ki_v_l = data_wave[1] * 0.0000001
        ki_v_r = ki_v_l
        
    if data_wave[2] != 0:
        kd_v_l = data_wave[2] * 0.00001
        kd_v_r = kd_v_l   
        
    if data_wave[3] != 0:
        pur_theita = data_wave[3]
        
    run_flag = data_wave[6]
    '''
    '''
    kp_dis = data_wave[0] * 0.001

    kd_dis = data_wave[1] * 0.001

    kp_w_z = data_wave[2] * 1
    ki_w_z = data_wave[3] * 0.01
        
    pur_v = data_wave[4]
    '''
    
    kp_1 = data_wave[0] * 0.001

    kd_1 = data_wave[1] * 0.001

    kp_2 = data_wave[2] * 0.001
    
    kd_2 = data_wave[3]
        
    pur_v = data_wave[4]
    
    if data_wave[5] != 0:
        ccd_limit = data_wave[5]
        
    if data_wave[7] != 0:
        pur_theita = data_wave[7]
        
    run_flag = data_wave[6]
    
    #gc.collect()









