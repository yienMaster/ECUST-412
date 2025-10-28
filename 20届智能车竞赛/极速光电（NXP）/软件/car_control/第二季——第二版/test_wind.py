from machine import *

import gc
import time

def cal_duty(x):
    return x * 65535 // 20000

dir = 1
up_time = 1000
pwm1 = PWM("C25", 50, cal_duty(up_time))

up_time = 1300

while True:
    time.sleep_ms(100)

    
    
    pwm1.duty_u16(cal_duty(up_time))





