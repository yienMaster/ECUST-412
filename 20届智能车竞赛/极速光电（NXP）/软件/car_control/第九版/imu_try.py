
# 本示例程序演示如何使用 seekfree 库的 MOTOR_CONTROLLER 类接口
# 使用 RT1021-MicroPython 核心板搭配 DRV8701/HIP4082 双驱模块进行测试

# 示例程序运行效果为电机反复正反加减速转动
# C4 LED 会根据电机的正反转点亮或熄灭

# 从 machine 库包含所有内容
from machine import *

# 从 seekfree 库包含 MOTOR_CONTROLLER
from seekfree import MOTOR_CONTROLLER
from smartcar import ticker
from seekfree import IMU963RA
from seekfree import WIRELESS_UART

# 包含 gc time 类
import gc
import time
import math

# 核心板上 C4 是 LED
led1 = Pin('C4' , Pin.OUT, pull = Pin.PULL_UP_47K, value = True)

# 实例化 MOTOR_CONTROLLER 电机驱动模块 一共四个参数 两个必填两个可选 [mode,freq,duty,invert]
# mode - 工作模式 一共四种选项 [PWM_C24_DIR_C26,PWM_C25_DIR_C27,PWM_C24_PWM_C26,PWM_C25_PWM_C27]
#        实际对应 DRV8701 双驱双电机 以及 HIP4082 双驱双电机 请确保驱动正确且信号连接正确
# freq - PWM 频率
# duty - 可选参数 初始的占空比 默认为 0 范围 ±10000 正数正转 负数反转 正转反转方向取决于 invert
# invert - 可选参数 是否反向 默认为 0 可以通过这个参数调整电机方向极性
motor_l = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C25_DIR_C27, 13000, duty = 0, invert = True)
motor_r = MOTOR_CONTROLLER(MOTOR_CONTROLLER.PWM_C24_DIR_C26, 13000, duty = 0, invert = True)

# 开发板上的 C19 是拨码开关
end_switch = Pin('C19', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
end_state = end_switch.value()

# 调用 IMU963RA 模块获取 IMU963RA 实例
# 参数是采集周期 调用多少次 capture 更新一次数据
# 可以不填 默认参数为 1 调整这个参数相当于调整采集分频
imu = IMU963RA()

ticker_flag = False
ticker_count = 0

# 定义一个回调函数 需要一个参数 这个参数就是 ticker 实例自身
def time_pit_handler(time):
    global ticker_flag  # 需要注意的是这里得使用 global 修饰全局属性
    global ticker_count
    ticker_flag = True  # 否则它会新建一个局部变量
    ticker_count = (ticker_count + 1) if (ticker_count < 100) else (1)

# 实例化 PIT ticker 模块 参数为编号 [0-3] 最多四个
pit1 = ticker(1)
# 关联采集接口 最少一个 最多八个 (imu, ccd, key...)
# 可关联 smartcar 的 ADC_Group_x 与 encoder_x
# 可关联 seekfree 的  IMU660RA, IMU963RA, KEY_HANDLER 和 TSL1401
pit1.capture_list(imu)
# 关联 Python 回调函数
pit1.callback(time_pit_handler)
# 启动 ticker 实例 参数是触发周期 单位是毫秒
pit1.start(1)

wireless = WIRELESS_UART(460800)

# 发送字符串的函数
wireless.send_str("Hello World.\r\n")
time.sleep_ms(500)

# data_analysis 数据解析接口 适配逐飞助手的无线调参功能
data_flag = wireless.data_analysis()
data_wave = [0,0,0,0,0,0,0,0]
for i in range(0,8):
    # get_data 获取调参通道数据 只有一个参数范围 [0-7]
    data_wave[i] = wireless.get_data(i)
    
    

#参数
motor_dir = 1
motor_duty = 0
motor_duty_max = 3000
theita = 0
omiga = 0
w_x = 0
w_z = 0
mx = 0
my = 0
mz = 0

while True:
    if (ticker_flag and ticker_count % 100 == 0):
        # 通过 capture 接口更新数据 但在这个例程中被 ticker 模块接管了
        # imu.capture()
        # 通过 get 接口读取数据
        imu_data = imu.get()
        mx = imu_data[6] + 3216
        my = imu_data[7] - 24 
        mz = imu_data[8] + 1181
        '''
        print("acc = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[0], imu_data[1], imu_data[2]))
        print("gyro = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[3], imu_data[4], imu_data[5]))
        print("mag = {:>6d}, {:>6d}, {:>6d}.".format(imu_data[6], imu_data[7], imu_data[8]))
        '''
        
        #俯仰角
        if(imu_data[1] == 0):
            a = 0.001
        else:
            a = imu_data[1]
        theita = math.atan2(imu_data[2] , a)
        
        w = (1/2) * math.pi - theita
        
        #偏航角
        b = my * math.cos(w) - mz * math.sin(w)
        
        if(mx == 0):
            c = 0.0001
        else:
            c = mx 
        omiga = math.atan2(b , c)
        
        #防止越界
        if theita <= 0:
            theita = 2 * math.pi + theita
       
            
        #print(omiga)

        
        #角速度
        w_x_dps = imu_data[3]
        w_z_dps = imu_data[4] * math.sin(theita) + imu_data[5] * math.cos(theita)
        
        #角速度换算
        trans = 250 / 65535 * 2**1 * math.pi / 180
        w_x = - w_x_dps * trans
        w_z = w_z_dps * trans
        
        #print(w_z)
        
        time.sleep_ms(10)
    
        if motor_dir:
            motor_duty = motor_duty + 50
            if motor_duty >= motor_duty_max:
                motor_dir = 0
        else:
            motor_duty = motor_duty - 50
            if motor_duty <= -motor_duty_max:
                motor_dir = 1

        led1.value(motor_duty < 0)
        # duty 接口更新占空比 范围 ±10000
        motor_l.duty(motor_duty)
        motor_r.duty(motor_duty)
        
        ticker_flag = False
        
    data_flag = wireless.data_analysis()
    for i in range(0,8):
        # 判断哪个通道有数据更新
        if (data_flag[i]):
            # 数据更新到缓冲
            data_wave[i] = wireless.get_data(i)
            # 将更新的通道数据输出到 Thonny 的控制台
            print("Data[{:<6}] updata : {:<.3f}.\r\n".format(i,data_wave[i]))
            
    # send_oscilloscope 将最多八个通道虚拟示波器数据上传到逐飞助手
    # 不需要这么多数据的话就只填自己需要的 只有两个数据就只填两个参数
    wireless.send_oscilloscope(theita , omiga , w_x , w_z)
       
        
    if end_switch.value() != end_state:
        pit1.stop()
        print("Ticker stop.")
        break
    


