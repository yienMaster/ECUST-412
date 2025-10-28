
# 本示例程序演示如何通过 boot.py 文件进行 soft-boot 控制
# 使用 RT1021-MicroPython 核心板搭配对应拓展学习板的拨码开关控制

# 示例程序运行效果为复位后执行本文件 通过 C18 电平状态跳转执行 user_main.py 或进入 main.py

# 从 machine 库包含所有内容
from machine import *

# 包含 gc 与 time 类
import gc
import time

# 上电启动时间延时
time.sleep_ms(50)

# 如果拨码开关打开 对应引脚拉低 就启动用户文件
os.chdir("/flash")
execfile("turn_try.py")

