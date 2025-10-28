#引入包含位姿传感器和电机驱动的库函数
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

from seekfree import IMU963RA
from seekfree import MOTOR_CONTROLLER

import gc
import time
#import IMU_cal
import pid
import math
import matrix_cal as matr

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
    
    x_pred = matr.matrix_multiply(F, x)  # x_pred = F * x
    # 预测新的协方差
    p_pred = matr.matrix_multiply(F, matr.matrix_multiply(p, matr.matrix_transpose(F)))  # p_pred = F * P * F^T
    p_pred = matr.matrix_add(p_pred, Q)  # p_pred = p_pred + Q

    # 更新步骤
    # 计算卡尔曼增益
    H_p_pred = matr.matrix_multiply(H, p_pred)  # H * p_pred
    H_p_pred_HT = matr.matrix_multiply(H_p_pred, matrix_transpose(H))  # H * p_pred * H^T
    S = matr.matrix_add(H_p_pred_HT, R)  # S = H * p_pred * H^T + R
    S_inv = matr.matrix_inverse(S)  # S_inv = (S)^-1
    K = matr.matrix_multiply(p_pred, matr.matrix_multiply(matrix_transpose(H), S_inv))  # K = p_pred * H^T * S_inv

    # 更新状态估计
    y = matr.matrix_sub(z, matr.matrix_multiply(H, x_pred))  # y = z - H * x_pred
    x_est = matr.matrix_add(x_pred, matr.matrix_multiply(K, y))  # x_est = x_pred + K * y

    # 更新协方差矩阵
    I = matr.identity_matrix()  # 单位矩阵
    KH = matr.matrix_multiply(K, H)  # K * H
    p_est = matr.matrix_sub(p_pred, matr.matrix_multiply(KH, p_pred))  # p_est = p_pred - K * H * p_pred

    return x_est, p_est

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
电机初始化
'''
# MOTOR_CONTROLLER 电机驱动模块 一共四个参数 [mode,freq,duty,invert]
# mode - 工作模式  [PWM_C24_DIR_C26,PWM_C25_DIR_C27]
# freq - PWM 频率
# duty - 可选参数 初始的占空比 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# invert - 可选参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

motor_duty = 0
motor_duty_max = 10000

'''
位姿传感器初始化
'''
# 参数是采集周期 调用多少次 capture 更新一次数据,默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU963RA()

#卡尔曼滤波参数
x = [[0],[0]]
p = [[550,0],[0,550]]
F = [[1,0.004],[0,1]]
H = [[1,0],[0,1]]
Q = [[1e-4,0],[0,1e-3]]
R = [[0.01,0],[0,0.25]]

'''
定时器初始化
'''
def time_pit_handler_2(time):
    global v_l,v_r
    #global imu_data
    v_l = encoder_l.get()
    v_r = encoder_r.get()
    #imu_data = imu.get()

pit2 = ticker(1)
# 关联采集接口
pit2.capture_list(encoder_l, encoder_r,imu)
# 关联 Python 回调函数
pit2.callback(time_pit_handler_2)
# 每1ms触发一次回调函数
pit2.start(1)

'''
pid初始化
'''
pid_out = 0
pid_out_l = 0
pid_out_r = 0
#直立参数
#前倾为正，弧度制
pur_theita = 0

kp_theita_l = 0
kd_theita_l = 0
in_theita_last_l = 0

kp_theita_r = kp_theita_l
kd_theita_r = kd_theita_l
in_theita_last_r = 0

pur_w_y = 0

inte_w_y_limit = 6000

kp_w_y_l = 10000
ki_w_y_l = 140
integral_w_y_l = 0

kp_w_y_r = kp_w_y_l
ki_w_y_r = ki_w_y_l
integral_w_y_r = 0

#速度参数
#范围 ±10000
pur_v = 4000

inte_v_limit = 2000

kp_v_l = 5000
ki_v_l = 3000
integral_v_l = 0

kp_v_r = kp_v_l
ki_v_r = ki_v_l
integral_v_r = 0

while True:
    #time.sleep_ms(1)
    imu_data = imu.get()
    '''
    位姿解算和卡尔曼滤波
    '''
    
    if(imu_data[0] == 0):
        a = 0.001
    else:
        a = imu_data[0]
    theita = math.atan2(imu_data[2] , a)
    
    if(imu_data[7] == 0):
        b = 0.001
    else:
        b = imu_data[7]
    w_y_dps = imu_data[4]
    
    #单位换算
    trans = 250 / 65535 * 2**1 * math.pi / 180
    w_y = w_y_dps * trans + 0.000864
    
    #theita,w_y = IMU_cal.deal_imu_data(imu_data)

    z = [[theita] ,[w_y]]
    x, p = kalman_filter(x, p, z, F, H, Q, R)
    
    theita_real =  x[0][0]
    w_y_real = x[1][0]
    
    
    '''
    pid控制
    '''
    '''
    pur_v_l = pur_v + v_turn
    pur_v_r = pur_v - v_turn
    integral_v_l, pid_out_l = v_pid(pur_v_l, v_l, kp_v_l, ki_v_l, integral_v_l, inte_v_limit)
    integral_v_r, pid_out_r = v_pid(pur_v_r, v_r, kp_v_r, ki_v_r, integral_v_r, inte_v_limit)
    
    pur_theita_l = pid_out_l + pur_theita
    pur_theita_r = pid_out_r + pur_theita
    '''
    pur_theita_l = 0
    pur_theita_r = 0
    in_theita_last_l, pid_out_l = theita_pid(pur_theita_l, theita_real, kp_theita_l, kd_theita_l, in_theita_last_l)
    in_theita_last_r, pid_out_r = theita_pid(pur_theita_r, theita_real, kp_theita_r, kd_theita_r, in_theita_last_r)
    
    w_y_pur_l = pid_out_l + pur_w_y
    w_y_pur_r = pid_out_r + pur_w_y
    integral_w_y_l, pid_out_l = w_y_pid(w_y_pur_l, w_y_real, kp_w_y_l, ki_w_y_l, integral_w_y_l ,inte_w_y_limit)
    integral_w_y_r, pid_out_r = w_y_pid(w_y_pur_r, w_y_real, kp_w_y_r, ki_w_y_r, integral_w_y_l ,inte_w_y_limit)
    
    motor_duty_l = pid_out_l
    motor_duty_r = pid_out_r
    if(motor_duty_l > 10000):
        motor_duty_l = 10000
    if(motor_duty_l < -10000):
        motor_duty_l = -10000
    if(motor_duty_r > 10000):
        motor_duty_r = 10000
    if(motor_duty_r < -10000):
        motor_duty_r = -10000
        
    #更新电机PWM输出
        
    print(motor_duty_r)
    motor_l.duty(motor_duty_r)
    motor_r.duty(motor_duty_l)
    
    gc.collect()
