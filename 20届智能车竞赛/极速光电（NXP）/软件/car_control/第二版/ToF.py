
# 本示例程序演示如何使用 seekfree 库的 DL1B 类接口
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板与 DL1B 模块测试

# 示例程序运行效果为每 1000ms(1s) 通过 Type-C 的 CDC 虚拟串口输出信息
# 可以通过 C19 的电平状态来控制是否退出测试程序
# 如果看到 Thonny Shell 控制台输出 ValueError: Module init fault. 报错
# 就证明 DL1B 模块连接异常 或者模块型号不对 或者模块损坏
# 请检查模块型号是否正确 接线是否正常 线路是否导通 无法解决时请联系技术支持

# 从 machine 库包含所有内容
from machine import *

# 从 smartcar 库包含 ticker
from smartcar import ticker

# 从 seekfree 库包含 DL1B
from seekfree import DL1B

# 包含 display 库
from display import *

# 包含 gc 类
import gc
import time


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
# 清屏 参数是 RGB565 格式的颜色数据
lcd.clear(0x0000)

# 定义一个长度为10的列表来存储最新的tof_data
window_size = 10
tof_data_window = [0] * window_size  # 初始化列表为0，长度为window_size

# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

# 调用 DL1B 模块获取 IMU660RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
tof = DL1B()

global tof_data



# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(tof)
# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    tof_data = tof.get()
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(10)

# 需要注意的是 ticker 是底层驱动的 这导致 Thonny 的 Stop 命令在这个固件版本中无法停止它
# 因此一旦运行了使用了 ticker 模块的程序 要么通过复位核心板重新连接 Thonny
# 或者像本示例一样 使用一个 IO 控制停止 Ticker 后再使用 Stop/Restart backend 按钮
# V1.1.2 以上版本则可以直接通过 Stop/Restart backend 按钮停止 Ticker

while True:
    time.sleep_ms(100)
    tof_data = tof.get()
    # 将新数据添加到列表中，移除最旧的数据
    #print(tof_data)
    tof_data_window.append(tof_data)
    tof_data_window.pop(0)  # 保证列表的长度为window_size
    
    # 计算当前列表中数据的平均值
    average_tof_data = sum(tof_data_window) / len(tof_data_window)
        
    # 打印输出滑动窗口中的平均值
    #print("Average distance = {:>6.2f}".format(average_tof_data))
    lcd.str32(0,52,"dis={:>6.2f}".format(average_tof_data),0xFFFF)
        
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    #gc.collect()


