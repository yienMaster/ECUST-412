#引入包含位姿传感器和电机驱动的库函数
import math
from machine import *

from smartcar import ticker
from smartcar import encoder

from seekfree import IMU963RA
from seekfree import MOTOR_CONTROLLER

import gc
import time
import IMU_cal
import pid

'''
编码器初始化
'''
# 对应学习板的编码器接口 1/2
# 总共三个参数 两个必要参数一个可选参数 [pinA,pinB,invert]
# pinA - 编码器 A 相或 PLUS 引脚
# pinB - 编码器 B 相或 DIR 引脚
# invert - 可选参数 是否反向 可以通过这个参数调整编码器旋转方向数据极性
encoder_l = encoder("D0", "D1", True)
encoder_r = encoder("D2", "D3", True)

'''
电机初始化
'''
# MOTOR_CONTROLLER 电机驱动模块 一共四个参数 [mode,freq,duty,invert]
# mode - 工作模式  [PWM_C24_DIR_C26,PWM_C25_DIR_C27]
# freq - PWM 频率
# duty - 可选参数 初始的占空比 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# invert - 可选参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = False)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = False)

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
# 定义一个回调函数
def time_pit_handler(time):
    imu_data = imu.get()
    v_l = encoder_l.get()
    v_r = encoder_r.get()

pit1 = ticker(1)
# 关联采集接口
pit1.capture_list(imu，encoder_l, encoder_r)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 每1ms触发一次回调函数
pit1.start(4)

'''
pid初始化
'''
pid_out = 0
pid_out_l = 0
pid_out_r = 0
#直立参数
#前倾为正，弧度制
pur_theita = 0

kp_theita_l = 3800
kd_theita_l = 300
in_theita_last_l = 0

kp_theita_r = 3800
kd_theita_r = 300
in_theita_last_r = 0

pur_w_y = 0

inte_w_limit = 2000

kp_w_y_l = 3800
ki_w_y_l = 300
integral_w_y_l = 0

kp_w_y_r = 3800
ki_w_y_r = 300
integral_w_y_r = 0

#速度参数
#范围 ±10000
pur_v = 4000

inte_v_limit = 2000

kp_v_l = 1000
ki_v_l = 1000
integral_v_l = 0

kp_v_r = 3800
ki_v_r = 300
integral_v_r = 0

while True:
    time.sleep_ms(1)
    
    '''
    位姿解算和卡尔曼滤波
    '''
    theita,w_y = IMU_cal.deal_imu_data(imu_data)

    z = [[theita] ,[w_y]]
    x, p = IMU_cal.kalman_filter(x, p, z, F, H, Q, R)
    
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
    motor_l.duty(motor_duty_l)
    motor_r.duty(motor_duty_r)
    #gc.collect()
