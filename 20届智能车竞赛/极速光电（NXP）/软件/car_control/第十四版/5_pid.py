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
encoder_l = encoder("C0" , "C1" )
encoder_r = encoder("C2" , "C3" , True)

#CCD初始化
ccd = TSL1401(1)


#电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)

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
F = [[1, 0.005], [0, 1]]
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

def find_road(data, width):
    """
    差比和滤波处理CCD数据，并同时寻找道路边线
    """
    global left_edge , right_edge

    mid_road = 64
    left_edge = 64
    right_edge = 64
    find_r = 0
    find_l = 0
    
    for i in range(1, 53):
        
        left_point = 64 - i
        
        right_point = 64 + i
        
        if find_l == 0:
            l = data[left_point - 1]
            c = data[left_point]
            r = data[left_point + 1]
            
            # 计算差比和
            diff_ratio = (l - r) / (c + 1) * 255  # 避免除零
            
            # 记录大于100的点，并存储位置相对于64的偏移
            if abs(diff_ratio) > 40:
                if diff_ratio > 0:
                    if find_r == 0 :
                        right_edge = left_point
                if diff_ratio < 0:
                    left_edge = left_point
                    find_l = 1
                    
        if(find_r == 0):
            l = data[right_point - 1]
            c = data[right_point]
            r = data[right_point + 1]
            
            # 计算差比和
            diff_ratio = (l - r) / (c + 1) * 255  # 避免除零
            
            # 记录大于100的点，并存储位置相对于64的偏移
            if abs(diff_ratio) > 40:
                if diff_ratio > 0:
                    right_edge = right_point
                    find_r = 1
                if diff_ratio < 0:
                    if find_l == 0:
                        left_edge = right_point
                        
        if find_l and find_r:
            break
    
    if find_l == 0 and find_r == 0:
        if left_edge == 64 and right_edge == 64:
            if data[70] > 10 and data[74] > 10 and data[80] > 10:
                left_edge = -53 + 64
                right_edge = 53 + 64
            else:
                left_edge =  64
                right_edge =  64
        elif left_edge == 64 and right_edge != 64:
            left_edge = right_edge - width
        elif right_edge == 64 and left_edge != 64:
            right_edge = left_edge + width
        else:
            left_edge = 64
            right_edge = 64
            
    elif find_l and find_r == 0:
        right_edge = 53 + 64
        
    elif find_r and find_l == 0:
        left_edge = -53 + 64
        
    road_width = right_edge - left_edge if right_edge > left_edge else 0
    if road_width <= 20:
        left_edge = 64 - 53
        right_edge = 64 + 53
    return left_edge - 64 , right_edge - 64 , road_width

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

kp_1 = 10 * 0.01
kp_2 = 3 * 0.01
kd_1 = 2 * 0.01
kd_2 = 3


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

inte_v_limit = 20000

kp_v_l = 440 * 0.00001
ki_v_l = 440 * 0.00000005
kd_v_l = 80 * 0.00001

kp_v_r = kp_v_l
ki_v_r = ki_v_l
kd_v_r = kd_v_l 

in_v_last_l = 0
in_v_last_r = 0
integral_v_r = 0
integral_v_l = 0

#直立参数
#前倾为正，弧度制
pur_theita = 2.2756

kp_theita_l = 0.9
kd_theita_l = 0.152
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 25

kp_w_y_l = 8050
ki_w_y_l = 800
kd_w_y_l = 2000
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
kd_w_y_r = kd_w_y_l
integral_w_y_r = 0
in_w_y_last_l = 0
in_w_y_last_r = 0

stop_flag = 0
#pid函数
def v_pid(pur, data, kp, ki, kd, inte,  in_last , v_limit):

    in_now =  pur - data
    
    if in_now >= 40:
        in_now = 40
    elif in_now <= -40:
        in_now = -40
        
    integral = inte + in_now
    
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral
    
    return integral, in_now, out


def theita_pid(pur, data, kp, kd, in_last):

    in_now = data - pur
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out


def w_y_pid(pur, data, kp, ki, kd, inte,  in_last ,w_y_limit):

    in_now = pur - data
    integral = inte + in_now
    
    if integral > w_y_limit:
        integral = w_y_limit
    elif integral < -w_y_limit:
        integral = -w_y_limit
        
    out = kp * in_now + ki * integral +  kd * (in_now - in_last)
    
    return integral, in_now, out

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
        
    out = kp1 * in_now + abs( in_now ) * in_now * kp2 + (in_now - in_last) * kd1 - gyro * kd2
    
    in_last = in_now
    
    if out >= 100:
        out = 100
    elif out <= -100:
        out = -100
    
    return in_last, out


def stand( theita_l, theita_r, v_l, v_r, theita_real, w_y_real):
    '''
    直立环
    '''
    global stop_flag
    global in_theita_last_l, in_theita_last_r, integral_w_y_l, integral_w_y_r, in_w_y_last_l,in_w_y_last_r
    
    '''前进环'''
    
    pur_theita_l = pur_theita + theita_l
    pur_theita_r = pur_theita + theita_r
    #pur_theita_l = pur_theita
    #pur_theita_r = pur_theita

    in_theita_last_l, pid_out_l = theita_pid(pur_theita_l, theita_real, kp_theita_l, kd_theita_l, in_theita_last_l)
    in_theita_last_r, pid_out_r = theita_pid(pur_theita_r, theita_real, kp_theita_r, kd_theita_r, in_theita_last_r)
    
    pur_w_y_l = -pid_out_l
    pur_w_y_r = -pid_out_r
    
    integral_w_y_l,in_w_y_last_l, pid_out_l = w_y_pid(pur_w_y_l, w_y_real, kp_w_y_l, ki_w_y_l, kd_w_y_l, integral_w_y_l ,in_w_y_last_l, inte_w_y_limit)
    integral_w_y_r,in_w_y_last_r, pid_out_r = w_y_pid(pur_w_y_r, w_y_real, kp_w_y_r, ki_w_y_r, kd_w_y_r, integral_w_y_r ,in_w_y_last_r, inte_w_y_limit)
    
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
    #wireless.send_oscilloscope(motor_duty_r,kp_theita_l,kd_theita_l,kp_w_y_l,ki_w_y_l, kd_w_y_l, pur_theita,theita_real)
    #wireless.send_oscilloscope(motor_duty_r,pur_theita,theita_real)
    #wireless.send_oscilloscope(motor_duty_l,pur_theita_l,theita_real, pur_w_y_l, w_y_real)
    #wireless.send_oscilloscope(v_l , integral_v_l , pur_v , pur_theita , kp_v_l * 100000 , ki_v_l * 10000000 , kd_v_l * 100000)
    #wireless.send_oscilloscope(v_turn)
    #wireless.send_oscilloscope(kp_theita_l,kd_theita_l,kp_w_y_l,ki_w_y_l, pur_theita , kp_v_l * 100000 , ki_v_l * 10000000)
    #wireless.send_oscilloscope(v_l, v_r, pur_v, integral_v_l , v_turn)
    
    if(stop_flag == 0):
        motor_l.duty(motor_duty_l + 120)
        motor_r.duty(motor_duty_r)
    else:
        motor_l.duty(0)
        motor_r.duty(0)
    
def follow_road(line_pos,  w_z_real , v_l, v_r):
    '''
    pid车辆循迹
    '''
    #global in_dis_last, integral_w_z, v_turn
    
    global in_dis_last
    global integral_v_l, integral_v_r , in_v_last_l, in_v_last_r
    '''转向环'''
    
    '''
    in_dis_last, pur_w_z = dis_pid(line_pos, 0, kp_dis, kd_dis, in_dis_last)
    integral_w_z, v_turn = w_z_pid(pur_w_z, w_z_real, kp_w_z, ki_w_z, integral_w_z ,inte_w_z_limit)
    '''
    
    in_dis_last, v_turn = follow_line(line_pos, 0, w_z_real , kp_1, kd_1, kp_2, kd_2, in_dis_last)
    #wireless.send_oscilloscope(v_turn, kp_dis, kd_dis, kp_w_z, ki_w_z, line_pos, integral_w_z)
    #wireless.send_oscilloscope(v_turn, pur_theita , kp_1, kd_1, kp_2, kd_2, line_pos, w_z_real)
    #wireless.send_oscilloscope(width_n,width_m,width_f)
    '''
    if v_turn <= -100:
        v_turn = -100
        
    if v_turn >= 100:
        v_tutn = 100
    
    k = v_turn * 0.01
    
    if k >= 0:
        pur_v_l = pur_v * (1 - k) /2 + pur_v /2
        pur_v_r = pur_v * (1 + 0.5 * k)/2 + pur_v /2
    else:
        pur_v_l = pur_v * (1 - 0.5 * k)/2 + pur_v /2
        pur_v_r = pur_v * (1 + k)/2 + pur_v /2
    '''
    
    if v_turn >= 0:
        pur_v_l = pur_v - v_turn
        pur_v_r = pur_v + v_turn /2
    else:
        pur_v_l = pur_v - v_turn /2
        pur_v_r = pur_v + v_turn
        
        
    #pur_v_l = pur_v - v_turn
    #pur_v_r = pur_v + v_turn
    
    #pur_v_l = pur_v
    #pur_v_r = pur_v
    
    integral_v_l, in_v_last_l, pid_out_l = v_pid(pur_v_l, v_l, kp_v_l, ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
    integral_v_r, in_v_last_r, pid_out_r = v_pid(pur_v_r, v_r, kp_v_r, ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
    
    return pid_out_l, pid_out_r

'''
定时器初始化
'''

cal_flag = True
ticker_count = 0
inte_flag = 0
del_flag = 0
theita_cal = 0
w_y_cal = 0
v_l = 0
v_r = 0
v_turn = 0
omiga_cal = 0
w_z_cal = 0
run_flag = 0
imu_data = imu.get()
follow_l = 0
follow_r = 0
follow_l_last = 0
follow_r_last = 0

def time_pit_handler_1(time):
    global x, p, ticker_count, v_l, v_r, cal_flag
    global follow_l, follow_r, follow_l_last, follow_r_last
    
    
    v_l = encoder_r.get()
    v_r = encoder_l.get()
    
    '''
    位姿解算和卡尔曼滤波
    '''
    
    ticker_count = (ticker_count + 1) if (ticker_count < 4) else (1)
    
    #位姿传感器读数
    imu_data = imu.get()
    
    #位姿解算
    if(imu_data[1] == 0):
        a = 0.001
    else:
        a = imu_data[1]
    theita = math.atan2(imu_data[2] , a)
    
    w_y_dps = imu_data[3] - 3
    
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
    theita_add_l = ticker_count * follow_l / 4
    theita_add_r = ticker_count * follow_r / 4
    
    #wireless.send_oscilloscope(theita_add_l, follow_l_last, follow_l, v_l, pur_v)
    #wireless.send_oscilloscope(v_l, v_r, follow_l, follow_r)
    
    if ticker_count == 3:
        cal_flag = 1
    
    if run_flag:
        
        #print(line_pos, w_z_cal)
        stand(theita_add_l, theita_add_r, v_l, v_r, theita_cal, w_y_cal)
        
        
def time_pit_handler_2(time):
    global v_l, v_r, line_pos
    global cal_flag, w_z_cal
    global follow_l, follow_r, follow_l_last, follow_r_last
    
    #wireless.send_oscilloscope(v_l, v_r)
        
    follow_l_last = follow_l
    follow_r_last = follow_r
    
    follow_l, follow_r = follow_road(line_pos,  w_z_cal, v_l, v_r)

        
pit1 = ticker(1)
pit2 = ticker(2)

# 关联采集接口
pit1.capture_list(imu, encoder_l, encoder_r)
pit2.capture_list( ccd)

# 关联 Python 回调函数
pit1.callback(time_pit_handler_1)
pit2.callback(time_pit_handler_2)

# 每5ms触发一次回调函数
pit1.start(5)
pit2.start(20)

straight_road_width_n = 87
straight_road_width_m = 71
straight_road_width_f = 55
total_road_width = 106

mode_1 = 1
mode_2 = 1
mode_3 = 0
mode_4 = 0
mode_pre = 1

last_left_n = -53
last_right_n = 53
last_left_m = -53
last_right_m = 53
last_left_f = -53
last_right_f = 53

sign_str = 0

sign_tur_l = 0
sign_tur_r = 0
sign_tur_ch = 0

sign_sci = 0
sci_num = 0
sign_sci_l = 0
sign_sci_r = 0

sign_sci_n = 1
sign_sci_m = 1
sign_sci_f = 1

sci_road_width_n_in = 0.5 * straight_road_width_n + 0.5 * total_road_width
sci_road_width_m_in = 0.5 * straight_road_width_m + 0.5 * total_road_width
sci_road_width_f_in = 0.5 * straight_road_width_f + 0.5 * total_road_width

sci_road_width_n_out = 0.5 * straight_road_width_n + 0.43 * total_road_width
sci_road_width_m_out = 0.5 * straight_road_width_m + 0.43 * total_road_width
sci_road_width_f_out = 0.5 * straight_road_width_f + 0.43 * total_road_width

sign_cro = 0
cro_num = 0
sign_cro_n = 1
sign_cro_m = 1
sign_cro_f = 1
sign_cro_out = 0

bar_road_width_f = 0.7 * straight_road_width_f
bar_road_width_m = 0.7 * straight_road_width_m
bar_road_width_n = 0.7 * straight_road_width_n

sign_bar = 0
sign_bar_l = 0
sign_bar_r = 0
sign_bar_out = 0
sign_bar_n = 0
sign_bar_m = 0
sign_bar_f = 0

sign_ram = 0
sign_err = 0

road_width_n = 0
road_width_m = 0
road_width_f = 0

#函数
while True:
    #读取传感器   
    if cal_flag :
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data1 = ccd.get(2)
        ccd_data2 = ccd.get(1)
        ccd_data3 = ccd.get(0)
        
        #75 60
        left_n, right_n, road_width_n = find_road(ccd_data1 , road_width_n)
        #wireless.send_oscilloscope(road_width_n,mode_1,sign_err)
        #print(left_n, right_n)
        left_m, right_m, road_width_m = find_road(ccd_data2 , road_width_m)
        left_f, right_f, road_width_f = find_road(ccd_data3 , road_width_f)
        
        wireless.send_oscilloscope(road_width_n, road_width_m, road_width_f)
        #wireless.send_oscilloscope(left_n, right_n, left_m,right_m,left_f,right_f,line_pos)
        '''
        状态机
        '''
        if sign_err == 0:   
            if road_width_n == 0:
                sign_err = 1    
        elif sign_err <= 4:
            if road_width_n == 0:
                sign_err += 1         
        elif sign_err == 5:
            sign_err = 0
            mode_1 = 3
            
            
        if mode_1 != 1:
            #环岛检测
            if (abs(road_width_f - total_road_width) <= 5):
                sign_sci = 0
                sign_sci_l = 0
                sign_sci_r = 0
                
            if sign_sci == 0:
                if (abs(road_width_f - sci_road_width_f_in) <= 5) and (abs(road_width_m - straight_road_width_m) <= 4) and (abs(road_width_n - straight_road_width_n) <= 4):
                    sign_sci = 1
                    
                    if left_f <= -48  and (abs(right_f - straight_road_width_f / 2) <= 2):
                        sign_sci_l = 1
                    elif right_f >= 48 and (abs(left_f + straight_road_width_f / 2) <= 2):
                        sign_sci_r = 1
                    else:
                        sign_sci = 0
                        
            elif sign_sci <= 2:
                
                if (abs(road_width_f - sci_road_width_f_in) <= 5) :
                    if sign_sci_l and (abs(right_f - straight_road_width_f / 2) <= 5):
                        sign_sci += 1
                    elif sign_sci_r and (abs(left_f + straight_road_width_f / 2) <= 5):
                        sign_sci += 1
                    else:
                        sign_sci = 0
                    
                elif (abs(road_width_f - straight_road_width_f) <= 5):
                    sign_sci = 0
                    sign_sci_l = 0
                    sign_sci_r = 0
                    
            else:
                if (abs(road_width_m - sci_road_width_m_in) <= 5):
                    if (abs(road_width_f - total_road_width) <= 10):
                        sign_sci = 0
                        sign_sci_l = 0
                        sign_sci_r = 0
                    else:
                        sign_sci = 0
                        mode_1 = 2
                        mode_2 = 1
                    
                        if sign_sci_l == 1:
                            mode_3 = 1
                            sign_sci_l = 0
                        elif sign_sci_r == 1:
                            mode_3 = 2
                            sign_sci_r = 0
                    
                    
            #十字检测  
            if sign_cro == 0:
                if abs(road_width_f - total_road_width) <= 3:
                    sign_cro = 1
                    
            elif 1 <= sign_cro <= 2:
                if abs(road_width_f - total_road_width) <= 3:
                    sign_cro += 1
                else:
                    sign_cro = 0
                    
            elif sign_cro == 3:
                
                cro_num += 1
                
                if cro_num == 100:
                    sign_cro = 0
                    cro_num = 0
                    
                if abs(road_width_m - total_road_width) <= 3:
                    sign_cro = 0
                    cro_num = 0
                    mode_1 = 2
                    mode_2 = 2
                    
             #障碍物检测
            if sign_bar == 0 :
                if (abs(road_width_f - bar_road_width_f) <= 5) :
                    sign_bar = 1
                    if (abs(right_f - straight_road_width_f/2) <= 2):
                        sign_bar_l = 1
                    elif (abs(left_f + straight_road_width_f/2) <= 2):
                        sign_bar_r = 1
                    else:
                        sign_bar = 0
            elif 1 <= sign_bar <= 2:
                if (abs(road_width_f - bar_road_width_f) <= 5) :
                    sign_bar += 1
                elif abs(road_width_f - straight_road_width_f) <= 3:
                    sign_bar = 0
            else:
                if abs(road_width_m - bar_road_width_m) <= 5:
                    sign_bar = 0
                    mode_1 = 2
                    mode_2 = 3
                    sign_bar_f = 1
                    sign_bar_m = 1
                    if sign_bar_l == 1:
                        mode_3 = 1
                        sign_bar_l = 0
                    elif sign_bar_r == 1:
                        mode_3 = 2
                        sign_bar_r = 0
                elif abs(road_width_f - straight_road_width_f) <= 3:
                    sign_bar = 0
                        
        #具体状态
        if mode_1 == 1:
            #直行状态
            if mode_2 == 1:
                line_pos = (0.4 * (left_n + right_n) + 0.50 * (left_m + right_m) + 0.1 * (left_f + right_f))*(1/2) 

        elif mode_1 == 2:
            
            if mode_2 == 1:
                
                if mode_3 == 1:
                    
                    if mode_4 == 0:
                        if abs(road_width_n - sci_road_width_n_in) <= 3:
                            sci_num += 1
                            if sci_num == 2:
                                sci_num = 0
                                mode_4 = 1
                            
                    elif mode_4 == 1:
                        
                        if sign_sci_f == 1:
                            
                            if abs(road_width_f - straight_road_width_f) <= 5:
                                sci_num += 1
                                if sci_num == 2:
                                    sci_num = 0
                                    sign_sci_f = 0
                        else:
                            
                            if sign_sci_m == 1:
                                
                                if abs(road_width_m - straight_road_width_m) <= 5:
                                    sci_num += 1
                                    if sci_num == 2:
                                        sci_num = 0
                                        sign_sci_m = 0
                            
                            else:
                                
                                if sign_sci_n == 1:
                                    if abs(road_width_n - straight_road_width_n) <= 5:
                                        sci_num += 1
                                        if sci_num == 2:
                                            sci_num = 0
                                            sign_sci_n = 0
                                            mode_4 = 2
                            
                            
                    elif mode_4 == 2:

                        if sci_num <= 3:
                            if road_width_n >= sci_road_width_n_in:
                                sci_num += 1
                                
                        elif sci_num >= 4:
                            if road_width_n <= sci_road_width_n_in:
                                sci_num += 1
                                if sci_num == 5:
                                    sci_num = 0
                                    mode_4 = 3
                                            
                    elif mode_4 == 3:
                        
                            
                        if sci_num <= 3:
                            
                            if sci_num == 0:
                                if (right_f - last_right_f >= 15):
                                    sci_num += 1
                            else:
                                if abs(total_road_width - road_width_f) <= 20:
                                    sci_num += 1
                                    
                                if sci_num == 3:
                                    sci_num += 1
                                    sign_sci_f = 0
                                    
                        else:
                            
                            if 4 <= sci_num <= 6:
                                
                                if sci_num == 4:
                                    if (right_m - last_right_m >= 10):
                                        sci_num += 1
                                else:
                                    if abs(total_road_width - road_width_m) <= 20:
                                        sci_num += 1
                                        
                                    if sci_num == 6:
                                        sci_num += 1
                                        sign_sci_m = 0
                            
                            else:
                                
                                if 7 <= sci_num <= 9:
                                    if sci_num == 7:
                                        if (right_n - last_right_n >= 7):
                                            sci_num += 1
                                    else:
                                        if abs(total_road_width - road_width_n) <= 20:
                                            sci_num += 1
                                            
                                        if sci_num == 10:
                                            sci_num = 0
                                            sign_sci_n = 0
                                            mode_4 = 4
                            
                            
                    elif mode_4 == 4:
                        if sign_sci_f == 0:
                            
                            if abs(road_width_f - sci_road_width_f_out) <= 5:
                                sci_num += 1
                                if sci_num == 2:
                                    sci_num = 0
                                    sign_sci_f = 1
                        else:
                            
                            if sign_sci_m == 0:
                                
                                if abs(road_width_m - sci_road_width_m_out) <= 5:
                                    sci_num += 1
                                    if sci_num == 2:
                                        sci_num = 0
                                        sign_sci_m = 1
                            
                            else:
                                
                                if sign_sci_n == 0:
                                    if abs(road_width_n - sci_road_width_m_out) <= 5:
                                        sci_num += 1
                                        if sci_num == 2:
                                            sci_num = 0
                                            sign_sci_n = 1
                                            mode_4 = 5
                    
                    elif mode_4 == 5:
                        if sign_sci_f == 1:
                            
                            if abs(road_width_f - straight_road_width_f) <= 3:
                                    sign_sci_f = 0
                        else:
                            
                            if sign_sci_m == 1:
                                
                                if abs(road_width_m - straight_road_width_m) <= 3:
                                    sign_sci_m = 0
                            
                            else:
                                
                                if sign_sci_n == 1:
                                    if abs(road_width_n - straight_road_width_n) <= 3:
                                        sci_num += 1
                                        if sci_num == 2:
                                            sci_num = 0
                                            sign_sci_n = 1
                                            sign_sci_m = 1
                                            sign_sci_f = 1
                                            mode_1 = 1
                                            mode_2 = 1
                                            mode_3 = 0
                                            mode_4 = 0
                                            
                    if mode_4 != 3 and mode_4 != 5:
                        
                        if sign_sci_n == 0:
                            right_n = left_n + straight_road_width_n
                        else:
                            left_n = right_n - straight_road_width_n
                        
                        if sign_sci_m == 0:
                            right_m = left_m + straight_road_width_m
                        else:
                            left_m = right_m - straight_road_width_m
                        
                        if sign_sci_f == 0:
                            right_f = left_f + straight_road_width_f
                        else:
                            left_f = right_f - straight_road_width_f
                        
                    elif mode_4 == 3:
                        right_n = left_n + straight_road_width_n
                        right_m = left_m + straight_road_width_m
                        right_f = left_f + straight_road_width_f
                        
                    elif mode_4 == 5:
                        left_n = right_n - straight_road_width_n
                        left_m = right_m - straight_road_width_m
                        left_f = right_f - straight_road_width_f
                        
                    line_pos = (0.2 * (left_n + right_n) + 0.5 * (left_m + right_m) + 0.3 * (left_f + right_f))*(1/2)
                #elif mode_3 == 2:
            
            elif mode_2 == 2:
                if abs(road_width_f - total_road_width) <= 3:
                    sign_cro_f = 0
                else:
                    sign_cro_f = 1
                    
                if abs(road_width_m - total_road_width) <= 3:
                    sign_cro_m = 0
                else:
                    sign_cro_m = 1
                    
                if abs(road_width_n - total_road_width) <= 3:
                    sign_cro_out = 1
                    sign_cro_n = 0
                else:
                    sign_cro_n = 1
                    
                if sign_cro_n and sign_cro_out:
                    sign_cro_out = 0
                    mode_1 = 1
                    mode_2 = 1
                
                total_cro = 0.7 * sign_cro_n  + 0.5 * sign_cro_m + 0.1 * sign_cro_f
                if total_cro != 0:
                    line_pos = (0.7 * (left_n + right_n) * sign_cro_n  + 0.5 * (left_m + right_m) * sign_cro_m + 0.1 * (left_f + right_f) * sign_cro_f)/ ( total_cro * 2)
                else:
                    line_pos = (0.28 * (left_n + right_n) + 0.47 * (left_m + right_m) + 0.25 * (left_f + right_f)) * (1/2)
            elif mode_2 == 3:
                
                if abs(road_width_f - straight_road_width_f) <= 3:
                    sign_bar_f = 0
                    
                if abs(road_width_m - straight_road_width_m) <= 3:
                    sign_bar_m = 0
                    
                if abs(road_width_n - bar_road_width_n) <= 3:
                    sign_bar_n = 1
                    sign_bar_out = 1
                    
                if abs(road_width_n - straight_road_width_n) <= 3:
                    sign_bar_n = 0
                 
                if mode_3 == 1:
                    if sign_bar_f == 1:
                        right_f = left_f + straight_road_width_f
                    
                    if sign_bar_m == 1:
                        right_m = left_m + straight_road_width_m
                    
                    if sign_bar_n == 1:
                        right_n = left_n + straight_road_width_n
                        
                elif mode_3 == 2:
                    if sign_bar_f == 1:
                        left_f = right_f - straight_road_width_f
                    
                    if sign_bar_m == 1:
                        left_m = right_m - straight_road_width_m
                    
                    if sign_bar_n == 1:
                        left_n = right_n - straight_road_width_n
                        
                if sign_bar_out and sign_bar_n == 0:
                    sign_bar_f = 0
                    sign_bar_m = 0
                    sign_bar_n = 0
                    sign_bar_out = 0
                    mode_1 = 1
                    mode_2 = 1
                    
                line_pos = (0.28 * (left_n + right_n) + 0.47 * (left_m + right_m) + 0.25 * (left_f + right_f)) * (1/2)
            
        elif mode_1 == 3:
            stop_flag = 1
        
        last_left_n = left_n
        last_right_n = right_n
        last_left_m = left_m
        last_right_m = right_m
        last_left_f = left_f
        last_right_f = right_f

        #wireless.send_oscilloscope(mode_1,mode_2,mode_3, mode_4, sign_cro_n, sign_cro_m, sign_cro_f)
        #wireless.send_oscilloscope(road_width_n, road_width_m, road_width_f,  mode_4,sign_sci_n,sign_sci_m,sign_sci_f)
        #wireless.send_oscilloscope(road_width_n, road_width_m, road_width_f, mode_2, mode_3, sign_sci)
        #wireless.send_oscilloscope(mode_1,mode_2,mode_3, mode_4, road_width_n, road_width_m, road_width_f,v_turn)
        #wireless.send_oscilloscope(line_pos)
        #wireless.send_oscilloscope(left_n, right_n, left_m, right_m, left_f, right_f,mode_1,mode_2)
        #位姿解算
        mx = imu_data[6] + 6275
        my = imu_data[7] + 123 
        mz = imu_data[8] - 132
        
        w = (1/2) * math.pi - theita_cal
        
        #偏航角
        mx_g = my * math.cos(w) - mz * math.sin(w)
        
        if(mx == 0):
            my_g = 0.0001
        else:
            my_g = mx 
        omiga = math.atan2(mx_g , my_g)
        
        w_z_dps = (imu_data[4] + 8 )* math.sin(theita_cal) + (imu_data[5] + 4 )* math.cos(theita_cal)
        
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
            if i == 6 or i == 4:
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
    
    kd_w_y_l = data_wave[4]
    kd_w_y_r = kd_w_y_l
    '''
    '''
    if data_wave[4] != 0:
        pur_theita = data_wave[4]
    '''
    '''
    pur_v = data_wave[3]
    '''
    '''
    #if data_wave[0] != 0:
        kp_v_l = data_wave[7] * 0.00001
        kp_v_r = kp_v_l
             
    #if data_wave[1] != 0:
        ki_v_l = data_wave[7] * 0.00000005
        ki_v_r = ki_v_l
    '''
    '''
#if data_wave[2] != 0:
    kd_v_l = data_wave[2] * 0.00001
    kd_v_r = kd_v_l   
    
    if data_wave[4] != 0:
        pur_theita = data_wave[4]
        
    run_flag = data_wave[5]
    '''
    '''
    kp_dis = data_wave[0] * 0.001

    kd_dis = data_wave[1] * 0.001

    kp_w_z = data_wave[2] * 1
    ki_w_z = data_wave[3] * 0.01
        
    pur_v = data_wave[4]
    '''
    '''
    kp_1 = data_wave[0] * 0.001

    kd_1 = data_wave[1] * 0.001

    kp_2 = data_wave[2] * 0.001
    
    kd_2 = data_wave[3]
    '''
    
    pur_v = data_wave[4]
    
    if data_wave[5] != 0:
        pur_theita = data_wave[5]
        
    
    run_flag = data_wave[6]
    

    if data_wave[7] != 0:
        stop_flag = data_wave[7]

    #gc.collect()













