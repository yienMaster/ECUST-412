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

# 状态转移矩阵 F
F = [[1, 0.001], [0, 1]]

# 观测矩阵 H
H = [[1, 0], [0, 1]]

# 过程噪声协方差 Q
Q = [[1e-4, 0], [0, 1e-3]]

# 观测噪声协方差 R
R = [[0.01, 0], [0, 0.25]]

#CCD函数

# 二值化处理
def threshold_and_denoise(data):
    # 计算数据的均值作为阈值
    #threshold = sum(data) / len(data)
    # 阈值处理：大于阈值为100，小于阈值为0
    binary_data = [100 if value >= 250 else 1 for value in data]
    denoised_data = array.array('h', binary_data)  # 保持原始数据类型为 'h'
    # 处理后的结果
    return denoised_data

#道路识别
def ccd_road_find(ccd_data):
    n = len(ccd_data)
    if n == 0:
        return 0, 0, 0
    mid_point = n // 2

    # 检测所有连续白色区域（100的区域）
    def find_white_regions(data):
        regions = []
        start = None
        for i in range(len(data)):
            if data[i] == 100:
                if start is None:
                    start = i
            else:
                if start is not None:
                    regions.append((start, i - 1))
                    start = None
        # 处理最后一个区域
        if start is not None:
            regions.append((start, len(data) - 1))
        return regions

    # 根据中心点选择主区域
    def select_main_region(regions, mid):
        # 优先选择包含中心点的区域
        for region in regions:
            start, end = region
            if start <= mid <= end:
                return region
        # 若无，则选择优先级最高的区域（距离近、宽度大、起始点靠右）
        selected = None
        for region in regions:
            start, end = region
            width = end - start + 1
            # 计算到中心点的最小距离
            dist_start = abs(start - mid)
            dist_end = abs(end - mid)
            current_dist = min(dist_start, dist_end)
            # 优先级元组：距离小、宽度大、起始点大（元组越小越优先）
            priority = (current_dist, -width, -start)
            if selected is None or priority < selected[1]:
                selected = (region, priority)
        return selected[0] if selected else None

    regions = find_white_regions(ccd_data)
    if not regions:
        return 0, 0, 0
    main_region = select_main_region(regions, mid_point)
    if not main_region:
        # 默认选第一个区域（极少数情况）
        main_region = regions[0]
    left, right = main_region
    return left, right, right - left

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
pur_theita = 2.2403

kp_theita_l = 0.514
kd_theita_l = 0.91
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 500

kp_w_y_l = 26000
ki_w_y_l = 725
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
integral_w_y_r = 0

#pid函数
def v_pid(pur, data, kp, ki, inte, v_limit):
    '''
    PID 控制直立
    
    参数:
    pur : 目标速度
    data : 当前速度
    kp : 比例系数
    ki : 积分系数
    inte :积分累计
    v_limit :积分上限
    
    返回:
    integral: 积分累计
    out : 目标角度
    '''
    in_now =  pur - data
    integral = inte + in_now
    
    if integral > v_limit:
        integral = v_limit
    elif integral < -v_limit:
        integral = -v_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out 


def theita_pid(pur, data, kp, kd, in_last):
    '''
    PID 控制theita
    
    参数:
    pur : 目标速度
    data : 当前角度
    kp : 比例系数
    kd : 微分系数
    in_last : 上一时刻的控制输入
    
    返回:
    in_now : 当前时刻的控制输入，用于下一次调用
    out : 目标角速度
    '''
    in_now = pur - data
    
    out = kp * in_now + kd * (in_now - in_last)
    
    return in_now, out


def w_y_pid(pur, data, kp, ki, inte ,w_y_limit):
    '''
    PID 控制w_y
    
    参数:
    pur : 目标角速度
    data : 当前角速度
    kp : 比例系数
    ki : 积分系数
    inte :积分累计
    w_y_limit :积分上限
    
    返回:
    integral : 积分累计
    out : 目标PWM的变化量
    '''
    in_now = data - pur
    integral = inte + in_now
    
    if integral > w_y_limit:
        integral = w_y_limit
    elif integral < -w_y_limit:
        integral = -w_y_limit
        
    out = kp * in_now + ki * integral
    
    return integral, out

def dis_pid(pur, data, kp, kd, in_last):
    '''
    PID 控制dis
    
    参数:
    pur : 道路线
    data : CCD中点
    kp : 比例系数
    kd : 微分系数
    in_last : 上一时刻的控制输入
    
    返回:
    in_now : 当前时刻的控制输入，用于下一次调用
    out : 目标角速度
    '''
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
    
    pur_theita_l = pur_theita
    pur_theita_r = pur_theita
    
    '''
    直立环
    '''
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
    wireless.send_oscilloscope(motor_duty_l,motor_duty_r,kp_theita_l,kd_theita_l,kp_w_y_l,ki_w_y_l,pur_theita,integral_w_y_l)
    motor_l.duty(motor_duty_l)
    motor_r.duty(motor_duty_r)

'''
定时器初始化（含有tof和ccd）
'''
cal_flag = True
ticker_count = 0
theita_cal = 0
w_y_cal = 0
run_flag = True

def time_pit_handler_1(time):
    global v_l, v_r ,theita_cal, w_y_cal
    global cal_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    
    ticker_count = (ticker_count + 1) if (ticker_count < 5) else (1)
    
    if ticker_count == 5:
        if run_flag:
            v_l = encoder_l.get() 
            v_r = encoder_r.get()
            cal_flag = True
            follow_road(line_pos, v_l, v_r, theita_cal, w_y_cal)
   
pit = ticker(1)
# 关联采集接口
pit.capture_list(encoder_l, encoder_r, imu , ccd ,tof)
# 关联 Python 回调函数
pit.callback(time_pit_handler_1)
# 每1ms触发一次回调函数
pit.start(1)

'''
状态机
'''
'''
mode_1
    1: 常规状态（直行和转向）
        mode_2
            1：直行 sign_str
            2：过弯 sign_tur
                mode_3
                    1: 左转
                    2：右转
      2：特殊状态（环岛，十字，障碍，斜坡）
        mode_2
            1：环岛 sign_sci
                mode_3
                    1: 左转
                    2：右转
            2：十字 sign_cro
            3：障碍 sign_bar
                mode_3
                    1: 左转
                    2：右转
            4：斜坡 sign_ram
            
            出特殊状态后，转到常规状态中的mode_pre模式（默认为1）
     3：异常状态
'''
#参数
straight_road_width_n = 85
straight_road_width_f = 41
total_road_width_f = 106
total_road_width_n = 106
road_width_bar = 28
road_width_sci = 74

mode_1 = 1
mode_2 = 1
mode_3 = 0
mode_4 = 0
mode_pre = 1

sign_str = 0

sign_tur_l = 0
sign_tur_r = 0
sign_tur_ch = 0

sign_sci = 0
sign_sci_l = 0
sign_sci_r = 0

sign_cro = 0
sign_cro_out = 0

sign_bar = 0
sign_bar_l = 0
sign_bar_r = 0
road_side_bar_l = 46
road_side_bar_r = 60

sign_ram = 0
sign_err = 0

#函数
while True:
    #读取传感器   
    if cal_flag :
        tof_data = tof.get()
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data1 = ccd.get(0)
        modified_list1 = ccd_data1[11:-11]
        ccd_data2 = ccd.get(1)
        modified_list2 = ccd_data2[11:-11]
        
        road_data1 = threshold_and_denoise(modified_list1)
        #road_data1 = threshold_and_denoise(modified_list1)
        road_data2 = threshold_and_denoise(modified_list2)
        
        road_l_f, road_r_f, road_width_f = ccd_road_find(processed_data2)
        road_l_n, road_r_n, road_width_n = ccd_road_find(processed_data1)
        
        '''
        状态机
        '''
        
        if mode_1 == 1：
            #环岛检测
            if sign_sci == 0 :
                if (abs(road_width_f - road_width_sci) <= 3):
                    sign_sci = 1
                    if (road_l_f <= 5):
                        sign_sci_l = 1
                    elif (road_r_f >= 100):
                        sign_sci_r = 1
            elif sign_sci == 1:
                if (abs(road_width_f - road_width_sci) <= 3) :
                    sign_sci += 1
                elif abs(road_width_f - total_road_width_f) <= 5:
                    sign_sci = 0
            else:
                if abs(road_width_f - total_road_width_f) <= 5:
                    sign_sci = 0
                    mode_1 = 2
                    mode_2 = 1
                    if sign_sci_l == 1
                        mode_3 = 1
                    elif sign_sci_r == 1
                        mode_3 = 2
                
            #十字检测
            if sign_cro == 0 :
                if abs(road_width_f - total_road_width_f) <= 5:
                    sign_cro = 1    
            elif abs(road_width_f - total_road_width_f) <= 5:
                if sign_cro <= 2:
                    sign_cro += 1         
            elif abs(road_width_f - total_road_width_f) >= 20:
                sign_cro = 0
            
            if sign_cro == 3:
                sign_cro = 0
                mode_1 = 2
                mode_2 = 2
                
            #障碍物检测
            if sign_bar == 0 :
                if (abs(road_width_f - road_width_bar) <= 3):
                    sign_bar = 1
                    if (abs(road_l_f  - road_side_bar_l) <= 5):
                        sign_bar_l = 1
                    elif (abs(road_r_f  - road_side_bar_r) <= 5):
                        sign_bar_r = 1
            elif sign_bar <= 2:
                if (abs(road_width_f - road_width_bar) <= 3) :
                    sign_bar += 1
                elif abs(road_width_f - total_road_width_f) <= 5:
                    sign_bar = 0
            else:
                if abs(road_width_f - total_road_width_f) <= 5:
                    sign_bar = 0
                    mode_1 = 2
                    mode_2 = 3
                    if sign_bar_l == 1
                        mode_3 = 1
                    elif sign_bar_r == 1
                        mode_3 = 2
                        
            #坡道检测
            if sign_ram == 0 :
                if (abs(road_width_f) >= 43) and (tof_data <= 500):
                    sign_ram = 1
                    
            elif sign_ram <= 2:
                if (abs(road_width_f) >= 43) and (tof_data <= 500):
                    sign_ram += 1
                else:
                    sign_ram = 0
            else:
                sign_ram = 0
                mode_1 = 2
                mode_2 = 4
                
            
        
        #故障检测
        if sign_err == 0:   
            if abs(road_width_n - total_road_width_n)<= 3:
                sign_err = 1    
        elif abs(road_width_n  - total_road_width_n) <= 3:
            if sign_err <= 4:
                sign_err += 1         
        elif abs(road_width_n) >= 10:
            sign_err = 0
        
        if sign_err == 5:
            sign_err = 0
            mode_1 = 3
        
        #具体状态
        if mode_1 == 1:
            road_center_n = (road_l_n + road_r_n) / 2
            road_center_f = (road_l_f + road_r_f) / 2
            
            #直行状态
            if mode_2 == 1:
                line_pos = road_center_n
            '''
                #直行——转向
                if (road_width_f - straight_road_width) >= 10:
                    
                    if (road_center_f - road_center_n) >= 10:
                        
                        if sign_tur_l == 0:
                            sign_tur_l = 1
                        else:
                            sign_tur_l += 1
                            
                            if sign_tur_l == 5:
                                mode_2 = 2
                                mode_3 = 1
                                
                if -(road_width_f - straight_road_width) >= 10:
                    
                    if -(road_center_f - road_center_n) >= 10:
                        
                        if sign_tur_r == 0:
                            sign_tur_r = 1
                        else:
                            sign_tur_r += 1
                            
                            if sign_tur_r == 5:
                                mode_2 = 2
                                mode_3 = 2
                                
            #转向状态
            elif mode_2 == 2:
                if mode_3 == 1:
                    #左转
                    line_pos = road_center_n - 0.25 * road_width_n
                    
                    #是否调换方向
                    if sign_tur_ch:
                        
                        if (road_center_f - road_center_n) < 10:
                            sign_tur_ch = 0
                        else:
                            sign_tur_ch += 1
                            if sign_tur_ch == 5:
                                mode_3 = 2
                                sign_tur_ch = 0
                    
                    if ((road_center_f - road_center_n) >= 10) and (sign_tur_ch == 0):
                        sign_tur_ch = 1
                        
                elif mode_3 == 2:
                    #右转
                    line_pos = road_center_n + 0.25 * road_width_n
                    
                    if sign_tur_ch:
                        if (road_center_n - road_center_f) < 10:
                            sign_tur_ch = 0
                        else:
                            sign_tur_ch += 1
                            if sign_tur_ch == 5:
                                mode_3 = 1
                                sign_tur_ch = 0
                    
                    if ((road_center_n - road_center_f) >= 10) and (sign_tur_ch == 0):
                        sign_tur_ch = 1
                
                #是否直行
                if abs(road_width_f - straight_road_width) <= 3:
                    sign_str += 1
                    if sign_str == 5:
                        mode_2 = 1
                        sign_str = 0
            
            follow_road(line_pos, w_z_real, v_l, v_r, theita_real, w_y_real)
            '''
        elif mode_1 == 2:
            if mode_2 == 1:
                if mode_3 == 1:
                    if mode_4 == 0:
                        line_pos = road_center_n
                        if abs(road_width_n - total_road_width_n) >= 8:
                            mode_4 = 1
                            
                    elif mode_4 == 1:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) <= 3:
                            mode_4 = 2
                            
                    elif mode_4 == 2:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) >= 8:
                            mode_4 = 3
                            
                    elif mode_4 == 3:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) <= 3:
                            mode_4 = 4
                            
                    elif mode_4 == 4:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) >= 8:
                            mode_4 = 5
                    
                    elif mode_4 == 5:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) <= 3:
                            mode_4 = 6
                            
                    elif mode_4 == 6:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) >= 8:
                            mode_4 = 7
                            
                    elif mode_4 == 7:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                       if abs(road_width_n - total_road_width_n) <= 3:
                            mode_1 = 1
                            mode_2 = 1
                        
                elif mode_3 == 2:
            
            if mode_2 == 2:
                if sign_cro_out == 0:
                    line_pos = road_center_n
                    
                    if abs(road_width_n - total_road_width_n) <= 3:
                        sign_cro_out = 1
                else:
                    line_pos = total_road_width_n / 2
                    if abs(road_width_n - straight_road_width_n) <= 5:
                        sign_cro_out = 0
                        mode_1 = 1
                        mode_2 = 1
                
            '''
            if mode_2 == 3:
                
            if mode_2 == 4:
            '''
        elif mode_1 == 3:
            run_flag = False
            motor_l.duty(0)
            motor_r.duty(0)
        
        #读取传感器   
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
        w_y = - w_y_dps * trans
        
        if theita <= 0:
            theita = 2 * math.pi + theita
        
        #卡尔曼滤波
        z = [theita,w_y]
        x, p = kalman_filter_explicit(x, p, z, F, H, Q, R)
        
        theita_cal =  x[0]
        w_y_cal = x[1] - 0.000327907906991503
        
        cal_flag = False
    
        

    #gc.collect()









