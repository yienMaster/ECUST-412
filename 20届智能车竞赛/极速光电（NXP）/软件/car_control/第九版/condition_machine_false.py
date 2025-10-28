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
# 定义片选引脚
cs = Pin('C5' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 拉高拉低一次 CS 片选确保屏幕通信时序正常
cs.high()
cs.low()
# 定义控制引脚
rst = Pin('B9' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
dc  = Pin('B8' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
blk = Pin('C4' , Pin.OUT, pull=Pin.PULL_UP_47K, value=1)
# 新建 LCD 驱动实例 这里的索引范围与 SPI 示例一致 当前仅支持 IPS200
drv = LCD_Drv(SPI_INDEX=1, BAUDRATE=60000000, DC_PIN=dc, RST_PIN=rst, LCD_TYPE=LCD_Drv.LCD200_TYPE)
# 新建 LCD 实例
lcd = LCD(drv)
# color 接口设置屏幕显示颜色 [前景色,背景色]
lcd.color(0xFFFF, 0x0000)
# mode 接口设置屏幕显示模式 [0:竖屏,1:横屏,2:竖屏180旋转,3:横屏180旋转]
lcd.mode(2)
# 清屏
lcd.clear(0x0000)


tof = DL1B(1)

'''
CCD初始化
'''
# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(1)

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


'''
定时器初始化（含有tof和ccd）
'''
ticker_count = 0
run_flag = True

def time_pit_handler_1(time):
    global cal_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    
    ticker_count = (ticker_count + 1) if (ticker_count < 5) else (1)
    
    if ticker_count == 5:
        cal_flag = True
   
pit = ticker(1)
# 关联采集接口
pit.capture_list( ccd ,tof)
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
    if True:
        tof_data = tof.get()
        '''
        CCD
        '''
        #CCD读数与处理
        ccd_data1 = ccd.get(0)
        #modified_list1 = ccd_data1[11:-11]
        ccd_data2 = ccd.get(1)
        #modified_list2 = ccd_data2[11:-11]
        
        road_data1 = threshold_and_denoise(modified_list1)
        #road_data1 = threshold_and_denoise(modified_list1)
        road_data2 = threshold_and_denoise(modified_list2)
        
        road_l_f, road_r_f, road_width_f = ccd_road_find(processed_data1)
        road_l_n, road_r_n, road_width_n = ccd_road_find(processed_data2)
        
        '''
        状态机
        '''
        
        if mode_1 == 1:
            #环岛检测
            if sign_sci == 0:
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
                    if sign_sci_l == 1:
                        mode_3 = 1
                    elif sign_sci_r == 1:
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
            '''    
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
                    if sign_bar_l == 1:
                        mode_3 = 1
                    elif sign_bar_r == 1:
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
        '''
            
        
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
                        
                #elif mode_3 == 2:
            
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
            #run_flag = False
            motor_l.duty(0)
            motor_r.duty(0)
        
        cal_flag = False
    
    '''
    lcd.str32(0, 0,"road_n={:>6.2f}".format(road_width_n),0xFFFF)
    lcd.str32(0, 30,"road_f={:>6.2f}".format(road_width_f),0xFFFF)
    lcd.str32(0, 60,"line_pos={:>6.2f}".format(line_pos),0xFFFF)
    lcd.str32(0, 90,"mode_1={:>6.2f}".format(mode_1),0xFFFF)
    lcd.str32(0, 120,"mode_2={:>6.2f}".format(mode_2),0xFFFF)
    lcd.str32(0, 150,"mode_3={:>6.2f}".format(mode_3),0xFFFF)
    lcd.str32(0, 180,"mode_4={:>6.2f}".format(mode_4),0xFFFF)
    '''
        

    #gc.collect()









