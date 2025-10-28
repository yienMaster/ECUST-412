
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
ccd = TSL1401(100)
tof = DL1B(1)

ticker_flag = False
ticker_count = 0
runtime_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)
    
# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
pit2 = ticker(2)
# 关联采集接口 最少一个 最多八个
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(ccd)
pit2.capture_list(tof)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
pit2.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(1)
pit2.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

#参数
processed_data1 = []
processed_data2 = []
modified_list1 = []
modified_list2 = []

while True:
    if (ticker_flag and ticker_count % 100 == 0):
        # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
        # ccd.capture()
        # 通过 get 接口读取数据 参数 [0,1] 对应学习板上 CCD1/2 接口
        ccd_data1 = ccd.get(0)
        ccd_data2 = ccd.get(1)
        tof_data = tof.get()
        
        #ccd处理
        processed_data1 = threshold_and_denoise(ccd_data1)
        processed_data2 = threshold_and_denoise(ccd_data2)
        
        '''
        # 去掉前11个数和后8个数
        modified_list1 = processed_data1[11:-8]
        modified_list2 = ccd_data1[11:-8]
        modified_list1 = processed_data1[11:-8]
        modified_list2 = ccd_data1[11:-8]
        '''
        
        #道路检测
        road_l_f, road_r_f, road_width_f = ccd_road_find(processed_data1)
        road_l_n, road_r_n, road_width_n = ccd_road_find(processed_data2)
        
        # 通过 wave 接口显示数据波形 (x,y,width,high,data,data_max)
        # x - 起始显示 X 坐标
        # y - 起始显示 Y 坐标
        # width - 数据显示宽度 等同于数据个数
        # high - 数据显示高度
        # data - 数据对象 这里基本仅适配 TSL1401 的 get 接口返回的数据对象
        # max - 数据最大值 TSL1401 的数据范围默认 0-255 这个参数可以不填默认 255
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
        print(f"左边界、右边界、宽度：{road_l_f, road_r_f, road_width_f}")
        print(f"左边界、右边界、宽度：{road_l_n, road_r_n, road_width_n}")
        '''    
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    gc.collect()