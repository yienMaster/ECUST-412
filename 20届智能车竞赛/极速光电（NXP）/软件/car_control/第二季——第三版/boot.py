
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 D8 电平状态跳转执行 user_main.py 或进入 main.py

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 上电启动时间延时
time.sleep_ms(50)
# 选择学习板上的一号拨码开关作为启动选择开关
boot_select = Pin('D8', Pin.IN, pull=Pin.PULL_UP_47K, value = True)
boot_select_2 = Pin('D9', Pin.IN, pull=Pin.PULL_UP_47K, value = True)

os.chdir("/flash")
# 如果拨码开关打开 对应引脚拉低 就启动用户文件
if boot_select.value() == 0:
    if boot_select_2.value() == 0:
        execfile("low_speed_with_key.py")
    else:
        execfile("use_for_all_high_speed.py")
else:
    if boot_select_2.value() == 0:
        execfile("test_with_key.py")
    else:
        execfile("desperado.py")
    
