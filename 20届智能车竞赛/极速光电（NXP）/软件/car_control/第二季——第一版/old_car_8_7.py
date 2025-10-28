#引入包含位姿传感器和电机驱动的库函数
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

from seekfree import IMU963RA
from seekfree import MOTOR_CONTROLLER
from seekfree import TSL1401
from seekfree import WIRELESS_UART
from seekfree import KEY_HANDLER

from display import *

import gc
import time
import math
import array

'''
初始化各个模块
'''

'''
#无线串口初始化
wireless = WIRELESS_UART(460800)

data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
for i in range(0,8):
    # get_data 获取调参通道数据 只有一个参数范围 [0-7]
    data_wave[i] = wireless.get_data(i)
'''
#编码器初始化
encoder_l = encoder("C0" , "C1" )
encoder_r = encoder("C2" , "C3" , True)

#CCD初始化
ccd = TSL1401(3)

#UART初始化
uart6 = UART(5)
uart6.init(115200)

key     = KEY_HANDLER(5)

#电机初始化
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C30_DIR_C31, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C28_DIR_C29, 13000, duty = 0, invert = True)

motor_duty_l = 0
motor_duty_r = 0
motor_duty_max = 7000

#位姿传感器初始化
imu = IMU963RA()

# 定义片选引脚
cs = Pin('B29' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B31', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C21', Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=2, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(1)
# 清屏 不传入参数就使用当前的 背景色 清屏
# 传入 RGB565 格式参数会直接把传入的颜色设置为背景色 然后清屏
lcd.clear(0x0000)

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
F = [[1, 0.044], [0, 1]]
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
            l = data[left_point - 2]
            c = data[left_point]
            r = data[left_point + 2]
            
            # 计算差比和
            diff_ratio = (l - r) / (c + 1) * 255  # 避免除零
            
            # 记录大于100的点，并存储位置相对于64的偏移
            if abs(diff_ratio) > 85:
                if diff_ratio > 0:
                    if find_r == 0 :
                        right_edge = left_point
                if diff_ratio < 0:
                    left_edge = left_point
                    find_l = 1
                    
        if(find_r == 0):
            l = data[right_point - 2]
            c = data[right_point]
            r = data[right_point + 2]
            
            # 计算差比和
            diff_ratio = (l - r) / (c + 1) * 255  # 避免除零
            
            # 记录大于100的点，并存储位置相对于64的偏移
            if abs(diff_ratio) > 85:
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
            #if data[70] > 10 or data[74] > 10 or data[80] > 10:
            left_edge = -53 + 64
            right_edge = 53 + 64
            '''
            else:
                left_edge =  64
                right_edge =  64
            '''
        elif left_edge == 64 and right_edge != 64:
            left_edge = right_edge - width
        elif right_edge == 64 and left_edge != 64:
            right_edge = left_edge + width
        else:
            left_edge = 64
            right_edge = 64
            
    elif find_l == 1 and find_r == 0 and right_edge == 64:
        right_edge = 53 + 64
        
    elif find_r == 1 and find_l == 0 and left_edge == 64:
        left_edge = -53 + 64
        
    road_width = right_edge - left_edge if right_edge > left_edge else 0
    '''
    if road_width <= 20:
        left_edge = 64 - 53
        right_edge = 64 + 53
    '''    
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

kp_turn = 190
kd_turn = 30

#速度参数
#范围 ±10000
pur_v = 245

inte_v_limit = 25000

kp_v_l = 450 * 0.00001
ki_v_l = 10 * 0.00000005
kd_v_l = 15 * 0.00001

kp_v_r = kp_v_l
ki_v_r = ki_v_l
kd_v_r = kd_v_l

v_out_l = 0
v_out_r = 0

in_v_last_l = 0
in_v_last_r = 0
integral_v_r = 0
integral_v_l = 0

#直立参数
#前倾为正，弧度制
pur_theita = 2.32

pur_theita_l = pur_theita
pur_theita_r = pur_theita
w_y_out_l = 0
w_y_out_r = 0

kp_theita_l = 0.21
kd_theita_l = 0.025
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 25

kp_w_y_l = 17000
ki_w_y_l = 1100
kd_w_y_l = 2800
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
kd_w_y_r = kd_w_y_l
integral_w_y_r = 0
in_w_y_last_l = 0
in_w_y_last_r = 0

pur_w_y_l = 0
pur_w_y_r = 0

stop_flag = 0
#pid函数
def v_pid(pur, data, kp, ki, kd, inte,  in_last , v_limit):

    in_now =  pur - data
    
    if abs(in_now) <= 30:
        integral = 0
        
    if in_now >= 30:
        in_now = 30
    elif in_now <= -30:
        in_now = -30
        
    integral = inte + in_now
        
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral + kd * (in_now - in_last)
    
    if out >= 4000:
        out = 4000
    elif out <= -4000:
        out = -4000
    
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

def follow_line(pur, data, gyro , kp, kd, d_corrcte_last, v_l, v_r, v_pur , in_last):

    in_now = pur - data
    offset = in_now * in_now * in_now / 7000 + in_now * 0.075
    d_corrcte = gyro * gyro / 1 + 1
    d_corrcte = 0.3 * d_corrcte_last + 0.7 * d_corrcte
    
    v_real = (v_l + v_r) / 2
    
#     out = ((v_real * v_real + 1)/ (v_pur * v_pur + 1) + 1 )* kp * offset - d_corrcte * kd * gyro
    out = math.log(1 + 0.3 * v_real ** 2 + 0.2 * v_pur ** 2) * kp * offset + 8500 * gyro +d_corrcte * kd * (in_now - in_last)

    #wireless.send_oscilloscope(in_now, offset ,d_corrcte, out, gyro)
    if out >= 4000:
        out = 4000
    elif out <= -4000:
        out = -4000
    
    return d_corrcte, in_last, out


'''
定时器初始化
'''

cal_flag = False
ticker_count = 0
turn_count = 0
inte_flag = 0
del_flag = 0
theita_cal = 0
w_y_now = 0
v_l = 0
v_r = 0
v_turn = 0
v_turn_last = 0
omiga_cal = 0
w_z_cal = 0
run_flag = 0
imu_data = imu.get()
w_y = 0
w_y_last = 0
theita_1 = 0
theita_2 = 2.2864
theita_2_flag = 0
theita_2_cre = 1
theita = 0
theita_1_last = 0
w_z_last = 0
w_z_real = 0
d_corrcte_last = 0
pur_theita_l_last = 0
pur_theita_r_last = 0
sign_i = 0
sign_stand = 0

stop_sign = 0
stop_num = 0
slow_flag = 0

theita_cal = 2.5
slow_count = 0


def time_pit_handler_1(time):
    global x, p, ticker_count, v_l, v_r, cal_flag ,theita_1, theita_2, w_y, theita, w_y_last,stop_sign
    global stop_flag, w_z_real, theita_1_last, turn_count, pur_theita_l_last, pur_theita_r_last
    global in_dis_last, v_turn, line_pos, d_corrcte_last,v_turn_last, sign_i,sign_stand,mode_1
    global integral_v_l, integral_v_r , in_v_last_l, in_v_last_r, pur_v, v_out_l, v_out_r, theita_cal
    global integral_w_y_l, integral_w_y_r, in_w_y_last_l,in_w_y_last_r, w_y_out_l, w_y_out_r
    global pur_theita_l, pur_theita_r, in_theita_last_l, in_theita_last_r, pur_w_y_l, pur_w_y_r, pur_theita
    global slow_flag, slow_count
    
    if slow_flag == 1:
        slow_count = slow_count + 1
        if slow_count == 500:
            slow_count = 0
            slow_flag = 0
            
    ticker_count = (ticker_count + 1) if (ticker_count < 10) else (1)
    turn_count = (turn_count + 1) if (turn_count < 3) else (1)
    
    '''
    位姿解算和卡尔曼滤波
    '''
    #wireless.send_oscilloscope(w_z)
    #位姿传感器读数
    imu_data = imu.get()
    
    #位姿解算
    if(imu_data[1] == 0):
        a = 0.001
    else:
        a = imu_data[1]
    theita_1 = math.atan2(imu_data[2] , a)

    w_y_dps = imu_data[3] - 3
    
    
    #单位换算
    trans = 250 / 65535 * 2**1 * math.pi / 180
    w_y = - w_y_dps * trans
    
    w_y = w_y_last + 0.8 * (w_y - w_y_last)
    
    if theita_1 <= 0:
        theita_1 = 2 * math.pi + theita_1
    
    theita_1 = theita_1_last + 0.8 * (theita_1 - theita_1_last)
    
    if theita_1 < 1.5:
        theita_1 = 1.5
    elif theita_1 > 2.62:
        theita_1 = 2.62
    
    #卡尔曼滤波
    z = [theita_1,w_y]
    x, p = kalman_filter_explicit(x, p, z, F, H, Q, R)
    
    theita_cal =  x[0]
    w_y_cal = x[1] + 0.000327907906991503
    #print(theita_cal)
    
    theita_1_last = theita_cal
    w_y_last = w_y_cal
    
    #wireless.send_oscilloscope(theita_cal, theita_1)
    
    
    if run_flag:
        if turn_count == 3:
            #in_dis_last, v_turn = follow_line(line_pos, 0, w_z_cal , kp_1, kd_1, kp_2, kd_2, in_dis_last)
            #print(1)
            d_corrcte_last,in_dis_last, v_turn = follow_line(line_pos, 0, w_z_real , kp_turn, kd_turn, d_corrcte_last, v_l, v_r, pur_v, in_dis_last)
            cal_flag = 1
            turn_count = 0
            if abs(v_turn_last - v_turn) <= 700:
                v_turn_last = v_turn
            elif abs(v_turn_last - v_turn) <= 1200:
                v_turn = 0.2 * v_turn_last + 0.8 * v_turn
                v_turn_last = v_turn
            elif abs(v_turn_last - v_turn) <= 1600:
                v_turn = 0.3 * v_turn_last + 0.7 * v_turn
                v_turn_last = v_turn
            else:
                if v_turn_last - v_turn > 0:
                    v_turn = v_turn_last - 1500
                    v_turn_last = v_turn
                else:
                    v_turn = v_turn_last + 1500
                    v_turn_last = v_turn
                    
            if v_turn >= 4000:
                v_turn = 4000
            elif v_turn <= - 4000:
                v_turn = -4000
                
            v_turn_last = v_turn
            
        if ticker_count % 10 == 0:

            if pur_v != 0:
                v_r = 0.8 * (encoder_l.get() + encoder_r.get()) / 2 + 0.2 * v_r
                v_l = v_r
            
            if pur_v != 0:
                if abs(v_l) >= pur_v + 120 or abs(w_z_real) > 20:
                    stop_flag = 1
                
                if mode_1 == 2:
                    real_pur_v = 165
                    
                if abs(line_pos) >= 8:
                    real_pur_v = 165
                else:
                    real_pur_v = 240
                    
                if v_l - real_pur_v >= -80:
                    sign_i = 1
                    integral_v_r = 0
                    integral_v_l = 0
                    
                if v_l >= 170:
                    real_pur_v = 140
                slow_flag = 0
                if slow_flag == 1:
                    real_pur_v = 140
            else:
                real_pur_v = 0
                
            if sign_i == 0:
                integral_v_l, in_v_last_l, v_out_l = v_pid(real_pur_v, v_l, 1.4 * kp_v_l, 10 * ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
                integral_v_r, in_v_last_r, v_out_r = v_pid(real_pur_v, v_r, 1.4 * kp_v_r, 10 * ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
            else:
                if slow_flag != 1:
                    if v_l <= 60:
                        integral_v_l, in_v_last_l, v_out_l = v_pid(real_pur_v, v_l, 2 * kp_v_l, 20 * ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
                        integral_v_r, in_v_last_r, v_out_r = v_pid(real_pur_v, v_r, 2 * kp_v_r, 20 * ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)    
                    elif 30 < v_l <= 170:    
                        integral_v_l, in_v_last_l, v_out_l = v_pid(real_pur_v, v_l, kp_v_l, ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
                        integral_v_r, in_v_last_r, v_out_r = v_pid(real_pur_v, v_r, kp_v_r, ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
                    else:
                        integral_v_l, in_v_last_l, v_out_l = v_pid(real_pur_v, v_l, 3 * kp_v_l, 25 * ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
                        integral_v_r, in_v_last_r, v_out_r = v_pid(real_pur_v, v_r, 3 * kp_v_r, 25 * ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
                else:
                    integral_v_l, in_v_last_l, v_out_l = v_pid(real_pur_v, v_l, 3 * kp_v_l, 25 * ki_v_l, kd_v_l, integral_v_l, in_v_last_l, inte_v_limit)
                    integral_v_r, in_v_last_r, v_out_r = v_pid(real_pur_v, v_r, 3 * kp_v_r, 25 * ki_v_r, kd_v_r, integral_v_r, in_v_last_r, inte_v_limit)
        if ticker_count % 2 == 0:
            pur_theita_l = pur_theita + v_out_l * round(ticker_count / 2 - 0.5) / 5
            pur_theita_r = pur_theita + v_out_r * round(ticker_count / 2 - 0.5) / 5
            
            in_theita_last_l, pur_w_y_l = theita_pid(pur_theita_l, theita_cal, kp_theita_l, kd_theita_l, in_theita_last_l)
            in_theita_last_r, pur_w_y_r = theita_pid(pur_theita_r, theita_cal, kp_theita_r, kd_theita_r, in_theita_last_r)
            pur_w_y_l = -pur_w_y_l
            pur_w_y_r = -pur_w_y_r

        integral_w_y_l,in_w_y_last_l, w_y_out_l = w_y_pid(pur_w_y_l, w_y_cal, kp_w_y_l, ki_w_y_l, kd_w_y_l, integral_w_y_l ,in_w_y_last_l, inte_w_y_limit)
        integral_w_y_r,in_w_y_last_r, w_y_out_r = w_y_pid(pur_w_y_r, w_y_cal, kp_w_y_r, ki_w_y_r, kd_w_y_r, integral_w_y_r ,in_w_y_last_r, inte_w_y_limit)
        w_y_out_l = -w_y_out_l
        w_y_out_r = -w_y_out_r
        
        #wireless.send_oscilloscope(kp_turn, kd_turn, v_turn)
        if w_y_out_l >= 7000:
            w_y_out_l = 7000
        elif w_y_out_l <= -7000:
            w_y_out_l = -7000
            
        if w_y_out_r >= 7000:
            w_y_out_r = 7000
        elif w_y_out_r <= -7000:
            w_y_out_r = -7000
        
        if v_turn < 0:
            motor_duty_l = w_y_out_l + v_turn * 1.5
            motor_duty_r = w_y_out_r - v_turn * 0.5
        else:
            motor_duty_l = w_y_out_l + v_turn * 0.5
            motor_duty_r = w_y_out_r - v_turn * 1.5
        #motor_duty_l = w_y_out_l + v_turn
        #motor_duty_r = w_y_out_r - v_turn

        if(motor_duty_l >= motor_duty_max):
            motor_duty_l = motor_duty_max
        elif(motor_duty_l <= -motor_duty_max):
            motor_duty_l = -motor_duty_max
        
        if(motor_duty_r >= motor_duty_max):
            motor_duty_r = motor_duty_max
        elif(motor_duty_r <= -motor_duty_max):
            motor_duty_r = -motor_duty_max
        
        if(stop_flag == 0 and stop_sign != 3):
            motor_l.duty(motor_duty_l)
            motor_r.duty(motor_duty_r)
        else:
            motor_l.duty(0)
            motor_r.duty(0)


key_flag     = False
key_count    = 0
choose_flag  = 1
sta_1 = 1
sta_2 = 0
sta_3 = 0
sta_4 = 1
sta_5 = 0

def time_pit_handler_2(time):
    global key_flag
    global key_count, run_flag, choose_flag, sta_1, sta_2, sta_3,sta_4,sta_5
    key_flag = True 
    key_count = (key_count + 1) if (key_count < 100) else (1)
    
    if key_count == 100:
        key_data = key.get()
        
        if key_data[2]:
            run_flag = 1
            key.clear(3)
        if key_data[3]:
            sta_5 = 0
            #sta_2 = 0
            choose_flag = 5
            key.clear(4)
        if key_data[0]:
            if sta_5 != 0:
                choose_flag = (choose_flag + 1) if (choose_flag < 5) else (1)
            key.clear(1)
        if key_data[1]:
            if choose_flag == 1:
                sta_1 = 0 if (sta_1 == 1) else (1)
            elif choose_flag == 2:
                sta_2 = 0 if (sta_2 == 1) else (1)
            elif choose_flag == 3:
                sta_3 = 0 if (sta_3 == 1) else (1)
            elif choose_flag == 4:
                sta_4 = 0 if (sta_4 == 1) else (1)
            else:
                sta_5 = 0 if (sta_5 == 1) else (1)
            key.clear(2)
            
pit1 = ticker(1)
pit2 = ticker(2)
# 关联采集接口
pit1.capture_list(imu, ccd)
pit2.capture_list(encoder_l, encoder_r, key)

# 关联 Python 回调函数
pit1.callback(time_pit_handler_1)
pit2.callback(time_pit_handler_2)

# 每5ms触发一次回调函数
pit1.start(3)
pit2.start(5)

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

road_width_n = 80
road_width_m = 60
road_width_f = 40
road_width_f_now = 80
road_width_m_now = 60
road_width_n_now = 40 

line_pos_last = 0
road_mid_n = 0
road_mid_m = 0
road_mid_f = 0
road_mid_n_last = 0
road_mid_m_last = 0
road_mid_f_last = 0

n = 0
m = 0.3
f = 0.7
m_l = 0.4
f_l = 0.6
m_r = 0.5
f_r = 0.5

#函数
while True:
    if (uart6.any()):
        buf = uart6.read()
        red_sign = buf.decode('utf-8')
    
    
    #wireless.send_oscilloscope(stop_sign,theita_cal)
    #读取传感器   
    if cal_flag :
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data2 = ccd.get(0)
        ccd_data3 = ccd.get(2)
        
        #75 60
        #wireless.send_oscilloscope(road_width_n,mode_1,sign_err)
        #print(left_n, right_n)
        left_m, right_m, road_width_m_now = find_road(ccd_data2 , road_width_m)
        left_f, right_f, road_width_f_now = find_road(ccd_data3 , road_width_f)
        
        #位姿解算
        mx = imu_data[6] + 6275
        my = imu_data[7] + 123 
        mz = imu_data[8] - 132

        w_z_dps = (imu_data[4] + 8 )* math.sin(theita) + (imu_data[5] + 4 )* math.cos(theita)
        
        #角速度换算
        trans = 250 / 65535 * 2**1 * math.pi / 180
        w_z_now = w_z_dps * trans
        w_z_real = w_z_last + 0.7 * (w_z_now - w_z_last)
        w_z_last = w_z_now

        #wireless.send_oscilloscope(v_l,v_r)

        if mode_1 == 1:
            #环岛检测
            if red_sign == "1":
                mode_1 = 2
                mode_2 = 1
                mode_3 = 0
                mode_4 = 0
                sci_num = 0
                slow_flag = 1
            
            if stop_sign == 0:
                if stop_num == 0:
                    if road_width_f_now <= 8:
                        stop_num = 1
                elif stop_num == 1:
                    if road_width_m_now <= 10:
                        stop_num = 2
                elif stop_num == 2:
                    if road_width_m_now >= 30:
                        stop_num = 0
                        stop_sign = 1
            
            elif stop_sign == 1:
                if stop_num <= 2:
                    if road_width_m_now <= 8:
                        stop_num = stop_num + 1
                    else:
                        stop_num = 0
                
                else:
                    stop_num = 0

            
            #障碍物检测
            if sta_4 == 1 and sign_bar == 0 :
                bar_road_width_f = road_width_f * 32.5 / 45
                bar_road_width_m = road_width_m * 32.5 / 45
                
                if (abs(road_width_f_now - bar_road_width_f) <= 8) :
                    sign_bar = 1
                    if (abs(right_f - last_right_f) <= 4):
                        sign_bar_l = 1
                    elif (abs(left_f - last_left_f) <= 4):
                        sign_bar_r = 1
                    else:
                        sign_bar = 0
            elif 1 <= sign_bar <= 2:
                if (abs(road_width_f_now - bar_road_width_f) <= 8) :
                    sign_bar += 1
                elif abs(road_width_f_now - road_width_f) <= 8:
                    sign_bar = 0
            else:
                if abs(road_width_m_now - bar_road_width_m) <= 8:
                    sign_bar = 0
                    mode_1 = 2
                    mode_2 = 2
                    if sign_bar_l == 1:
                        mode_3 = 1
                    elif sign_bar_r == 1:
                        mode_3 = 2
                        
        #具体状态
        if mode_1 == 1:
            #直行状态
            if mode_2 == 1:
                
                if abs(left_m - last_left_m ) >= 100:
                    left_m = last_left_m
                if abs(right_m - last_right_m ) >= 100:
                    right_m = last_right_m
                    
                if abs(left_f - last_left_f ) >= 100:
                    left_f = last_left_f
                if abs(right_f - last_right_f ) >= 100:
                    right_f = last_right_f
                
                 
                if abs(road_width_f_now - 50) < 5:
                    road_width_f = road_width_f_now
                
                if abs(road_width_m_now - 70) < 5:
                    road_width_m = road_width_m_now


                last_left_m = left_m
                last_right_m = right_m
                last_left_f = left_f
                last_right_f = right_f
                
                line_pos =  (m * (left_m + right_m) + f * (left_f + right_f))*(1/2)
                
                '''
                if abs((left_m + right_m) - (left_f + right_f)) >= 30:
                    line_pos = (left_m + right_m) * 0.5
                '''    
                if stop_num != 0:
                    line_pos = 0
                #wireless.send_oscilloscope(left_m, right_m,left_f, right_f,road_width_m, road_width_f)
                
                #wireless.send_oscilloscope(left_n, right_n, left_f, right_f, v_turn,v_l, v_r)

        elif mode_1 == 2:
            #环岛
            if mode_2 == 1:
                
                if mode_3 == 0:
                    sci_num = sci_num + 1
                    
                    if red_sign == "0":
                        mode_1 = 1
                        mode_2 = 1
                        mode_3 = 0
                        mode_4 = 0
                        sci_num = 0
                        
                    sci_road_width_m_in = 0.5 * road_width_m + 0.5 * total_road_width
                    sci_road_width_f_in = 0.5 * road_width_f + 0.5 * total_road_width
                    
                    if sta_2 == 1 and (((left_f - last_left_f)<= -15) and left_f <= -50 and abs(right_f-road_width_f/2)<10):
                        sign_sci_l = 1

                    elif sta_3 == 1 and (((right_f - last_right_f)>= 15) and right_f >= 50 and abs(left_f+road_width_f/2)<10):
                        sign_sci_r = 1

                    if sign_sci_l == 1:
                        mode_3 = 1
                        sign_sci_m = 1
                        sign_sci_f = 1
                        sign_sci_l = 0
                        sci_num = 0
                        
                    elif sign_sci_r == 1:
                        mode_3 = 2
                        sign_sci_m = 1
                        sign_sci_f = 1
                        sign_sci_r = 0
                        sci_num = 0
                        
                    else:
                        mode_1 = 1
                        mode_2 = 1
                        mode_3 = 0
                        mode_4 = 0
                        sci_num = 0
                    
                    if abs((left_m + right_m) - (left_f + right_f)) >= 10:
                        line_pos = (left_m + right_m) * 0.5
                        
                    else:
                        line_pos = (m * (left_m + right_m) + f * (left_f + right_f))*(1/2)
                    '''
                    line_pos = (m * (left_m + right_m) + f * (left_f + right_f))*(1/2)
                    '''
                elif mode_3 == 1:
                    
                    if mode_4 == 0:
                        
                        if sign_sci_f == 1:
                            
                            if abs(road_width_f_now - road_width_f) <= 10:
                                sci_num += 1
                                if sci_num == 2:
                                    sci_num = 0
                                    sign_sci_f = 0
                        else:
                            
                            if sign_sci_m == 1:
                                
                                if abs(road_width_m_now  - road_width_m) <= 10:
                                    sci_num += 1
                                    if sci_num == 2:
                                        sci_num = 0
                                        sign_sci_m = 0
                                        mode_4 = 1

                    elif mode_4 == 1:

                        if right_m == 53:
                            sci_num = 0
                            mode_4 = 2
                                    
                    elif mode_4 == 2:

                        if right_m < 53:
                            mode_4 = 3
                        
                    elif mode_4 == 3:

                        if red_sign == "1" and (road_width_f_now == 106 or road_width_m_now == 106):
                            sign_sci_f = 0
                            sign_sci_m = 0
                            sci_num = 0
                            mode_4 = 4

                    elif mode_4 == 4:
                        if right_f < 53  and red_sign == "0":
                            sign_sci_f = 1
                            sign_sci_m = 1
                            sci_num = 0
                            mode_4 = 5

                    elif mode_4 == 5:

                        if  (abs(road_width_m_now - 70) <= 10 and abs(left_m+right_m) <=15 and left_m > -50) and (abs(road_width_f_now - 50) <= 10 and abs(left_f+right_f) <= 15 and left_f > -40):
                            
                            sci_num = 0
                            mode_1 = 1
                            mode_2 = 1
                            mode_3 = 0
                            mode_4 = 0
                    
                    #wireless.send_oscilloscope(left_m, right_m,left_f, right_f,line_pos, mode_4,road_width_m,road_width_f)
                    
                    
                    if mode_4 != 3 and mode_4 != 4 and mode_4 != 5:
                        
                        last_left_m = left_m
                        last_right_m = right_m
                        last_left_f = left_f
                        last_right_f = right_f

                        if sign_sci_m == 0:
                            if left_m == -53:
                                right_m = left_m + road_width_m*1.2
                            else:
                                right_m = left_m + road_width_m*1.1
                        else:
                            if right_m == 53:
                                left_m = right_m - road_width_m*0.55
                            else:
                                left_m = right_m - road_width_m*0.65
                        
                        if sign_sci_f == 0:
                            if left_f == -53:
                                right_f = left_f + road_width_f*1.2
                            else:
                                right_f = left_f + road_width_f*1.1
                        else:
                            if right_f == 53:
                                left_f = right_f - road_width_f*0.55
                            else:
                                left_f = right_f - road_width_f*0.65
                        
                        
                    elif mode_4 == 3:
                        
                        if abs(left_m - last_left_m ) >= 100:
                            left_m = last_left_m
                        if abs(right_m - last_right_m ) >= 100:
                            right_m = last_right_m
                            
                        if abs(left_f - last_left_f ) >= 100:
                            left_f = last_left_f
                        if abs(right_f - last_right_f ) >= 100:
                            right_f = last_right_f

                        last_left_m = left_m
                        last_right_m = right_m
                        last_left_f = left_f
                        last_right_f = right_f
                    
                    elif mode_4 == 4:
                        line_pos = -18.5
                        
                    elif mode_4 == 5:
                        #if left_m < - 40:
                        
                        if right_m > 40:
                            left_m = right_m - 90
                        else:
                            left_m = right_m - 70

                        if right_f > 30:
                            left_f = right_f - 70
                        else:
                            left_f = right_f - 50
                        '''
                        left_m = right_m - 91
                        left_f = right_f - 71
                        '''
                    if mode_4 != 4:
                        line_pos = (m_l * (left_m + right_m) + f_l * (left_f + right_f))*(1/2)
                        
                        if mode_4 == 3 and line_pos > -5:
                            line_pos = -5
                        
                elif mode_3 == 2:
                    
                    if mode_4 == 0:
                        
                        if sign_sci_f == 1:
                            
                            if abs(road_width_f_now - road_width_f) <= 10:
                                sci_num += 1
                                if sci_num == 2:
                                    sci_num = 0
                                    sign_sci_f = 0
                        else:
                            
                            if sign_sci_m == 1:
                                
                                if abs(road_width_m_now  - road_width_m) <= 10:
                                    sci_num += 1
                                    if sci_num == 2:
                                        sci_num = 0
                                        sign_sci_m = 0
                                        mode_4 = 1

                    elif mode_4 == 1:
                        if left_m == -53:
                            sci_num = 0
                            mode_4 = 2
                                    
                    elif mode_4 == 2:
                        if left_m > -53:
                            mode_4 = 3
                        
                    elif mode_4 == 3:
                        if red_sign == "1" and (road_width_f_now == 106 or road_width_m_now == 106):
                            sign_sci_f = 0
                            sign_sci_m = 0
                            sci_num = 0
                            mode_4 = 4

                    elif mode_4 == 4:
                        if (left_f > -53 or left_m > -53) and red_sign == "0":
                            sign_sci_f = 1
                            sign_sci_m = 1
                            sci_num = 0
                            mode_4 = 5

                    elif mode_4 == 5:

                        if (abs(road_width_m_now - 70) <= 10 and abs(left_m+right_m) <=15 and right_m < 50) and (abs(road_width_f_now - 50) <= 10 and abs(left_f+right_f) <=15 and right_f < 40):
                            
                            sci_num = 0
                            mode_1 = 1
                            mode_2 = 1
                            mode_3 = 0
                            mode_4 = 0
                    
                    #wireless.send_oscilloscope(left_m, right_m,left_f, right_f,line_pos, mode_4,road_width_m,road_width_f)
                    
                    
                    if mode_4 != 3 and mode_4 != 4 and mode_4 != 5:
                        
                        last_left_m = left_m
                        last_right_m = right_m
                        last_left_f = left_f
                        last_right_f = right_f
                        
                        if sign_sci_m == 1:
                            if left_m == -53:
                                right_m = left_m + road_width_m*0.75
                            else:
                                right_m = left_m + road_width_m*0.85
                        else:
                            if right_m == 53:
                                left_m = right_m - road_width_m*1.05
                            else:
                                left_m = right_m - road_width_m*1
                        
                        if sign_sci_f == 1:
                            if left_f == -53:
                                right_f = left_f + road_width_f*0.75
                            else:
                                right_f = left_f + road_width_f*0.85
                        else:
                            if right_f == 53:
                                left_f = right_f - road_width_f*1.05
                            else:
                                left_f = right_f - road_width_f*1
                        
                        
                    elif mode_4 == 3:
                        if abs(left_m - last_left_m ) >= 100:
                            left_m = last_left_m
                        if abs(right_m - last_right_m ) >= 100:
                            right_m = last_right_m
                            
                        if abs(left_f - last_left_f ) >= 100:
                            left_f = last_left_f
                        if abs(right_f - last_right_f ) >= 100:
                            right_f = last_right_f

                        last_left_m = left_m
                        last_right_m = right_m
                        last_left_f = left_f
                        last_right_f = right_f
                    
                    elif mode_4 == 4:
                        line_pos = 18.5
                        
                    elif mode_4 == 5:
                        
                        if left_m < - 40:
                            right_m = left_m + 80
                        else:
                            right_m = left_m + 65
                            
                        if left_f < -30:
                            right_f = left_f + 60
                        else:
                            right_f = left_f + 45
                        '''
                        right_m = left_m + 52
                        right_f = left_f + 42
                        '''
                    if mode_4 != 4:
                        line_pos = (m_r * (left_m + right_m) + f_r * (left_f + right_f))*(1/2)
                        
                        if mode_4 == 3 and line_pos < 5:
                            line_pos = 5
                            
            elif mode_2 == 2:
                if abs(road_width_f_now - bar_road_width_f) <= 10:
                    sign_bar_f = 1
                else:
                    sign_bar_f = 0
                
                if sign_bar_m == 0:
                    
                    if abs(road_width_m_now - bar_road_width_m) <= 15:
                        sign_bar_m = 1
                        sign_bar_out = 1
                else:
                    if abs(road_width_m_now - road_width_m) <= 20:
                        sign_bar_m = 0
                
                if mode_3 == 1:
                    if sign_bar_f == 1:
                        right_f = left_f + 50
                    
                    if sign_bar_m == 1:
                        right_m = left_m + 70
                        
                elif mode_3 == 2:
                    if sign_bar_f == 1:
                        left_f = right_f - 50
                    
                    if sign_bar_m == 1:
                        left_m = right_m - 70
                        
                if sign_bar_out and sign_bar_m == 0:
                    sign_bar_out = 0
                    mode_1 = 1
                    mode_2 = 1
                    
                line_pos =  (m * (left_m + right_m) + f * (left_f + right_f))*(1/2)
                    
        cal_flag = False
     
    else:
        if sta_5 == 1:
            if choose_flag == 1:
                lcd.str32(0, 0,  "*", 0xFFFF)
                lcd.str32(0, 40,  " ", 0xFFFF)
                lcd.str32(0, 80,  " ", 0xFFFF)
                lcd.str32(0, 120,  " ", 0xFFFF)
                lcd.str32(0, 160,  " ", 0xFFFF)
            elif choose_flag == 2:
                lcd.str32(0, 0,  " ", 0xFFFF)
                lcd.str32(0, 40,  "*", 0xFFFF)
                lcd.str32(0, 80,  " ", 0xFFFF)
                lcd.str32(0, 120,  " ", 0xFFFF)
                lcd.str32(0, 160,  " ", 0xFFFF)
            elif choose_flag == 3:
                lcd.str32(0, 80,  "*", 0xFFFF)
                lcd.str32(0, 40,  " ", 0xFFFF)
                lcd.str32(0, 0,  " ", 0xFFFF)
                lcd.str32(0, 120,  " ", 0xFFFF)
                lcd.str32(0, 160,  " ", 0xFFFF)
            elif choose_flag == 4:
                lcd.str32(0, 80,  " ", 0xFFFF)
                lcd.str32(0, 40,  " ", 0xFFFF)
                lcd.str32(0, 0,  " ", 0xFFFF)
                lcd.str32(0, 120,  "*", 0xFFFF)
                lcd.str32(0, 160,  " ", 0xFFFF)
            elif choose_flag == 5:
                lcd.str32(0, 80,  " ", 0xFFFF)
                lcd.str32(0, 40,  " ", 0xFFFF)
                lcd.str32(0, 0,  " ", 0xFFFF)
                lcd.str32(0, 120,  " ", 0xFFFF)
                lcd.str32(0, 160,  "*", 0xFFFF)
            
            if sta_1 == 1:
                lcd.str32(15, 0,  "SPEED     =  HIGH", 0xFFFF)
            else:
                lcd.str32(15, 0,  "SPEED     =   LOW ", 0xFFFF)
            
            if sta_2 == 1:
                lcd.str32(15, 40, "LEFT_SCI  =    ON ", 0xFFFF)
            else:
                lcd.str32(15, 40, "LEFT_SCI  =   OFF", 0xFFFF)
                
            if sta_3 == 1:
                lcd.str32(15, 80, "RIGHT_SCI =    ON ", 0xFFFF)
            else:
                lcd.str32(15, 80,  "RIGHT_SCI =   OFF", 0xFFFF)
            
            if sta_4 == 1:
                lcd.str32(15, 120, "BAR       =    ON ", 0xFFFF)
            else:
                lcd.str32(15, 120,  "BAR       =   OFF", 0xFFFF)
                
            if sta_5 == 1:
                lcd.str32(15, 160, "SCREEN    =    ON ", 0xFFFF)
            else:
                lcd.str32(15, 160,  "SCREEN    =   OFF", 0xFFFF)
    '''
    调参用无线串口

    
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

    #run_flag = data_wave[6]
    
    #stop_flag = 0
    
    #gc.collect()

















 



