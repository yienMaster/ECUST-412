
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

# 从 seekfree 库包含 TSL1401
from seekfree import TSL1401

# 包含 gc 类
import gc

import array

count=[]

# 卷积操作
def convolve_1d(data, kernel):
    """对一维数据进行卷积操作。"""
    kernel_size = len(kernel)
    padding = kernel_size // 2
    result = array.array('h', [0] * len(data))  # 保持与原数据相同的类型和大小

    for i in range(len(data)):
        sum_val = 0
        for j in range(kernel_size):
            idx = i + j - padding
            if 0 <= idx < len(data):  # 保证索引在范围内
                sum_val += data[idx] * kernel[j]
        
        # 将卷积结果转换为整数并确保它不会超出短整型的范围
        result[i] = int(round(sum_val))  # 转换为整数类型

    return result

# 腐蚀操作
def erosion(data, structure):
    """对一维数据进行腐蚀操作。"""
    window_size = len(structure)
    padding = window_size // 2
    result = array.array('h', [0] * len(data))  # 保持与原数据相同的类型和大小

    for i in range(len(data)):
        min_val = float('inf')  # 腐蚀时取最小值
        for j in range(window_size):
            idx = i + j - padding
            if 0 <= idx < len(data):
                min_val = min(min_val, data[idx])
        result[i] = min_val

    return result

# 膨胀操作
def dilation(data, structure):
    """对一维数据进行膨胀操作。"""
    window_size = len(structure)
    padding = window_size // 2
    result = array.array('h', [0] * len(data))  # 保持与原数据相同的类型和大小

    for i in range(len(data)):
        max_val = -float('inf')  # 膨胀时取最大值
        for j in range(window_size):
            idx = i + j - padding
            if 0 <= idx < len(data):
                max_val = max(max_val, data[idx])
        result[i] = max_val

    return result

# 主函数，处理 CCD 数据
def process_ccd_data(ccd_data):
    # 使用 array('h') 格式的数据进行处理
    data = array.array('h', ccd_data)  # 确保数据是 array 类型

    # 卷积核：简单的均值滤波器（大小为3的窗口）
    kernel = [1, 1, 1]  # 卷积核
    kernel = [k / sum(kernel) for k in kernel]  # 归一化卷积核

    # 卷积操作
    convolved_data = convolve_1d(data, kernel)

    # 腐蚀操作（使用大小为3的窗口）
    structure = [1, 1, 1]  # 腐蚀和膨胀用相同的结构元素
    eroded_data = erosion(convolved_data, structure)

    # 膨胀操作（使用大小为3的窗口）
    dilated_data = dilation(eroded_data, structure)

    return convolved_data, eroded_data, dilated_data

#01化处理
def threshold_and_denoise(data):
    # 计算数据的均值作为阈值
    threshold = sum(data) / len(data)
    
    # 阈值处理：大于阈值为1，小于阈值为0
    binary_data = [100 if value > threshold else 1 for value in data]
    
    # 去噪点处理：如果两边为1，则中间点也设为1
    denoised_data = array.array('h', binary_data)  # 保持原始数据类型为 'h'
    
    for i in range(1, len(denoised_data) - 1):
        # 如果当前点为0，并且两边都为1，则当前点设置为1
        if denoised_data[i] == 1 and denoised_data[i - 1] == 100 and denoised_data[i + 1] == 100:
            denoised_data[i] = 100
        # 如果当前点为1，并且两边都为0，则当前点设置为0
        elif denoised_data[i] == 100 and denoised_data[i - 1] == 1 and denoised_data[i + 1] == 1:
            denoised_data[i] = 1
    
    # 处理后的结果
    return denoised_data

#道路提取
def ccd_road_find(ccd_data):
    '''
    提取道路，计算道路宽度
    '''
    #提取道路中点
    n = len(ccd_data)
    mid_point = n // 2
    
    boundary_points = []
    left = mid_point
    right = mid_point

    #中点向左右扫描交界点
    while left >= 0 or right < n:
        if (left > 0) and (ccd_data[left] != ccd_data[left - 1]):
            if (ccd_data[left] == ccd_data[left + 1]) and (ccd_data[left - 1] == ccd_data[left - 2]):
                if ccd_data[left] == 100:
                    boundary_points.append(left)
                else:
                    boundary_points.append(left - 1)
                
                if len(boundary_points) == 2:
                    break
        # 检查右侧
        if (right < n - 1) and (ccd_data[right] != ccd_data[right + 1]):
            if (ccd_data[right] == ccd_data[right - 1]) and (ccd_data[right + 1] == ccd_data[right + 2]):
                if ccd_data[right] == 100:
                    boundary_points.append(right)
                else:
                    boundary_points.append(right + 1)
                
                if len(boundary_points) == 2:
                    break
        
        # 向左右扫描
        right += 1
        left -= 1
        
    
    # 如果没有检测到两个交界点
    if len(boundary_points) < 2:
        
        subsequence = ccd_data1[n - 20: n - 1]
        avg = sum(subsequence) / len(subsequence)

        if avg > 90:
            boundary_points.append(n - 1)
       
        subsequence = ccd_data1[0: 19]
        avg = sum(subsequence) / len(subsequence)

        if avg > 90:
            boundary_points.append(0)
    
    # 计算两个交界点的距离
    if len(boundary_points) == 2:
        boundary_points.sort()
        distance = abs(boundary_points[1] - boundary_points[0])
        return boundary_points[0], boundary_points[1], distance
    else:
        return None, None, 0  # 如果没有找到交界点，返回None和0
    
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
ccd = TSL1401(10)

ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    ticker_flag = True  # 否则它会新建一个局部变量

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(ccd)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker
del_pos_l = [0] * 100
del_pos_r = [0] * 100
modified_list1 = []
modified_list2 = []
while True:
    # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
    # ccd.capture()
    # 通过 get 接口读取数据 参数 [0,1] 对应学习板上 CCD1/2 接口
    ccd_data1 = ccd.get(0)
    ccd_data2 = ccd.get(1)
    #先后对图像进行卷积、腐蚀、膨胀处理，dilated_data为最终结果
    '''
    ccd处理
    '''
    convolved_data, eroded_data, dilated_data = process_ccd_data(ccd_data1)
    processed_data1 = threshold_and_denoise(dilated_data)
    '''
    # 去掉前11个数和后8个数
    modified_list1 = processed_data1[11:-8]
    modified_list2 = ccd_data1[11:-8]
    '''
    convolved_data, eroded_data, dilated_data = process_ccd_data(ccd_data2)
    processed_data2 = threshold_and_denoise(dilated_data)
    '''
    # 去掉前11个数和后8个数
    modified_list1 = processed_data1[11:-8]
    modified_list2 = ccd_data1[11:-8]
    '''
    
    '''
    道路检测
    '''
    road_l_f, road_r_f, road_width_f = ccd_road_find(processed_data1)
    road_l_n, road_r_n, road_width_n = ccd_road_find(processed_data2)
    

    
    # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
    # x - 起始显示 X 坐标
    # y - 起始显示 Y 坐标
    # width - 数据显示宽度 等同于数据个数
    # high - 数据显示高度
    # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
    # max - 数据最大值 TSL1401 的数据范围默认 0-255 这个参数可以不填默认 255
    lcd.wave(0,  0, 128, 64, processed_data1, max = 255)
    lcd.str32(0,60,"width_f={:>6.2f}".format(road_width_f),0xFFFF)
    #lcd.wave(0, 80, 128, 64, ccd_data1, max = 255)
    lcd.wave(0, 160, 128, 64, processed_data2, max = 255)
    lcd.str32(0,240,"width_n={:>6.2f}".format(road_width_n),0xFFFF)
    #lcd.wave(0, 240, 128, 64, ccd_data2, max = 255)

    # 计算100次的平均位置（假设processed_data_list有100次数据）
    for i in range(128):  # 从0到127
        if processed_data1[i] == 100:
            del_pos_l.pop(0)
            del_pos_l.append(i)
            break
        
    for i in range(128):  # 从0到127
        if processed_data1[127 - i] == 100:
            del_pos_r.pop(0)
            del_pos_r.append(i)
            break
        
    pos_ave_l = sum(del_pos_l) / len(del_pos_l)
    pos_ave_r = sum(del_pos_r) / len(del_pos_r)
        
    pos_l = round(pos_ave_l)
    pos_r = round(pos_ave_r)

    print(f"平均左边位置：{pos_l}")
    print(f"平均右边位置：{pos_r}")
        
    #print("runtime_count = {:>6d}.".format(runtime_count))
    #print(processed_data)
    #print("Dilated Data:", dilated_data)
        
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()

