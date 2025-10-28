
# 本示例程序演示如何使用 seekfree 库的 TSL1401 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 TSL1401 IPS200 模块测试

# 示例程序运行效果是实时在 IPS200 屏幕上显示 CCD 的采集图像

# CCD 的曝光计算方式
# CCD 通过 TSL1401(x) 初始化构建对象时 传入的 x 代表需要进行几次触发才会更新一次数据
# Ticker 通过 start(y) 启动时 y 代表 Ticker 的周期
# 此时每 y 毫秒会触发一次 CCD 的更新
# 当触发次数大于等于 x 时 CCD 才会更新一次数据
# 因此 CCD 的曝光时间等于 y * x 本例程中就是 10ms * 10 = 100ms

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 包含 display 库
from display import *

# 外设库
from seekfree import TSL1401
from seekfree import DL1B

#数据库
import gc
import array

'''CCD'''
def find_road(data):
    """
    差比和滤波处理CCD数据，并同时寻找道路边线
    """
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
        if abs(diff_ratio) > 60:
            offset = i - 64
            if diff_ratio > 0:
                pos_values.append(offset)
            else:
                neg_values.append(offset)
    
    # 默认边界值
    left_edge, right_edge = -53, 53
    
    if neg_values:
        left_edge = min(neg_values, key=abs)
    if pos_values:
        right_edge = min(pos_values, key=abs)
    
    road_width = right_edge - left_edge if right_edge > left_edge else 0
    return left_edge , right_edge , road_width , filtered_data

# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

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

# 调用 TSL1401 模块获取 CCD 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 默认参数为 1 调整这个参数相当于调整曝光时间倍数
ccd = TSL1401(2)
tof = DL1B(1)

ticker_count = 0
run_flag = True
cal_flag = True

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
pit.start(5)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

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
straight_road_width_n = 74
straight_road_width_f = 38
total_road_width_f = 106
total_road_width_n = 106
road_width_bar = 28
road_width_sci_f = 70
road_width_sci_n = 85
road_width_sci_in = 55

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
sci_num = 0
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
        
processed_data1 = []
processed_data2 = []


while True:
    if cal_flag:
        '''
        CCD
        '''
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        tof_data = tof.get()
        
        #ccd处理+道路检测
        road_l_f, road_r_f, road_width_f , processed_data1 = find_road(ccd_data1)
        road_l_n, road_r_n, road_width_n , processed_data2 = find_road(ccd_data2)
        
        '''
        状态机
        '''
        
        if mode_1 == 1:
            #环岛检测
            if sign_sci == 0:
                if (abs(road_width_f - road_width_sci_f) <= 5):
                    sign_sci = 1
                    if (road_l_f <= 16):
                        sign_sci_l = 1
                    elif (road_r_f >= 100):
                        sign_sci_r = 1
                    else:
                        sign_sci = 0
            elif sign_sci == 1:
                if (abs(road_width_f - road_width_sci_f) <= 5) :
                    sign_sci += 1
                elif (abs(road_width_f - total_road_width_f) <= 5):
                    sign_sci = 0
            else:
                if road_width_sci_in >= road_width_f:
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
            
        '''
        #故障检测
        if sign_err == 0:   
            if abs(road_width_n)<= 3:
                sign_err = 1    
        elif abs(road_width_n) <= 3:
            if sign_err <= 4:
                sign_err += 1         
        elif abs(road_width_n) >= 10:
            sign_err = 0
        
        if sign_err == 5:
            sign_err = 0
            mode_1 = 3
        '''
        #具体状态
        if mode_1 == 1:
            road_center_n = ((road_l_n + road_r_n) / 2) - 64
            road_center_f = ((road_l_f + road_r_f) / 2) - 64
            
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
                        if abs(road_width_n - straight_road_width_n) >= 8:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 1
                            
                    elif mode_4 == 1:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) <= 3:
                            sci_num += 1
                            if sci_num == 5:
                                sci_num = 0
                                mode_4 = 2
                            
                    elif mode_4 == 2:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) >= 8:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 3
                            
                    elif mode_4 == 3:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) <= 3:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 4
                            
                    elif mode_4 == 4:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) >= 8:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 5
                    
                    elif mode_4 == 5:
                        road_r_n = road_l_n + straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) <= 3:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 6
                            
                    elif mode_4 == 6:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) >= 8:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_4 = 7
                            
                    elif mode_4 == 7:
                        road_l_n = road_r_n - straight_road_width_n
                        road_center_n = (road_l_n + road_r_n) / 2
                        if abs(road_width_n - straight_road_width_n) <= 3:
                            sci_num += 1
                            if sci_num == 3:
                                sci_num = 0
                                mode_1 = 1
                                mode_2 = 1
                    
                    line_pos = road_center_n
                        
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
            line_pos = 0
            #run_flag = False
            #motor_l.duty(0)
            #motor_r.duty(0)
        
        cal_flag = False
    '''
    
    lcd.wave(0, 0, 128, 64, processed_data1, max = 255)
    lcd.wave(0, 80, 128, 64, ccd_data1, max = 255)
    lcd.wave(0, 160, 128, 64, processed_data2, max = 255)
    lcd.wave(0, 240, 128, 64, ccd_data2, max = 255)
    
    lcd.str32(130, 0,"{:>6.2f}".format(road_l_f),0xFFFF)
    lcd.str32(130, 30,"{:>6.2f}".format(road_r_f),0xFFFF)
    lcd.str32(130, 60,"{:>6.2f}".format(road_width_f),0xFFFF)
    lcd.str32(130, 160,"{:>6.2f}".format(road_l_n),0xFFFF)
    lcd.str32(130, 190,"{:>6.2f}".format(road_r_n),0xFFFF)
    lcd.str32(130, 220,"{:>6.2f}".format(road_width_n),0xFFFF)
    
    lcd.str32(130, 250,"{:>6.2f}".format(tof_data),0xFFFF)

    
    '''
    lcd.str32(0, 0,"road_n={:>6.2f}".format(road_width_n),0xFFFF)
    lcd.str32(0, 30,"road_f={:>6.2f}".format(road_width_f),0xFFFF)
    lcd.str32(0, 60,"line_pos={:>6.2f}".format(line_pos - 64),0xFFFF)
    lcd.str32(0, 90,"mode_1={:>6.2f}".format(mode_1),0xFFFF)
    lcd.str32(0, 120,"mode_2={:>6.2f}".format(mode_2),0xFFFF)
    lcd.str32(0, 150,"mode_3={:>6.2f}".format(mode_3),0xFFFF)
    lcd.str32(0, 180,"mode_4={:>6.2f}".format(mode_4),0xFFFF)
    lcd.str32(0, 210,"sign_sci={:>6.2f}".format(sign_sci),0xFFFF)
    lcd.str32(0, 240,"road_l_f={:>6.2f}".format(road_l_f),0xFFFF)
    lcd.str32(0, 270,"road_r_f={:>6.2f}".format(road_r_f),0xFFFF)
    
    
    
