import sensor
import time
import math
import pyb
import image
from pyb import LED
from machine import UART

uart = UART(2, baudrate=115200)     # 初始化串口 波特率设置为115200

# threshold_index = 0

# 颜色跟踪阈值（L 最小值，L 最大值，A 最小值，A 最大值，B 最小值，B 最大值）
# 下面的阈值通常用于跟踪红色/绿色/蓝色的物体。您可能需要调整它们...
thresholds = [
    (61, 72, 39, 61, 70, 77), # 超强光红色
    (28, 56, 45, 79, 3, 60), # 正常光红色
    (0, 100, 35, 82, -5, 65), # 强光红色
    (9, 22, 22, 38, 13, 30), # 超暗光红色
    (15, 53, 39, 70, 23, 59), # 暗光红色
    (0, 100, 19, 64, 10, 51), # 超暗光红色
    (62, 86, 9, 45, 44, 92), # 橙色
    (51, 69, 56, 82, -37, -4) # 开窗红色
    ]

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False) # 自动增益
sensor.set_auto_whitebal(False) # 自动白平衡

# 只有像素数超过“pixel_threshold”且面积超过“area_threshold”的blob才会被
# “find_blobs”返回。如果更改了“pixels_threshold”和“area_threshold”，请进行相应调整。
# 相机分辨率。"merge=True" 合并图像中所有重叠的斑点。

'''
thresholds是颜色的阈值，注意：这个参数是一个列表，可以包含多个颜色。
如果你只需要一个颜色，那么在这个列表中只需要有一个颜色值，如果你想要多个颜色阈值，那这个列表就需要多个颜色阈值。
注意：在返回的色块对象blob可以调用code方法，来判断是什么颜色的色块。

roi是“感兴趣区”。图像坐标原点为左上角。
roi的格式是(x, y, w, h)的tupple.
x:ROI区域中左上角的x坐标
y:ROI区域中左上角的y坐标
w:ROI的宽度
h:ROI的高度
left_roi = [0,0,160,240]
blobs = img.find_blobs([red],roi=left_roi)

x_stride 就是查找的色块的x方向上最小宽度的像素，默认为2，如果你只想查找宽度10个像素以上的色块，那么就设置这个参数为10：
blobs = img.find_blobs([red],x_stride=10)

y_stride 就是查找的色块的y方向上最小宽度的像素，默认为1，如果你只想查找宽度5个像素以上的色块，那么就设置这个参数为5：
blobs = img.find_blobs([red],y_stride=5)

invert 反转阈值，把阈值以外的颜色作为阈值进行查找

area_threshold 面积阈值，如果色块被框起来的面积小于这个值，会被过滤掉

pixels_threshold 像素个数阈值，如果色块像素数量小于这个值，会被过滤掉

merge 合并，如果设置为True，那么合并所有重叠的blob为一个。
注意：这会合并所有的blob，无论是什么颜色的。如果你想混淆多种颜色的blob，只需要分别调用不同颜色阈值的find_blobs。

margin 边界，如果设置为1，那么两个blobs如果间距1一个像素点，也会被合并。

OpenMV 的IDE里加入了阈值选择工具，极大的方便了对于颜色阈值的调试。
首先运行hello world.py让IDE里的framebuffer显示图案。

然后打开 工具 → Mechine Vision → Threshold Editor

点击 Frame Buffer可以获取IDE中的图像，Image File可以自己选择一个图像文件。

拖动六个滑块，可以实时的看到阈值的结果，我们想要的结果就是，将我们的目标颜色变成白色，其他颜色全变为黑色。
'''
white = LED(4)

while True:
    img = sensor.snapshot().lens_corr(strength = 1.8, zoom = 1.0)
    white.on()
    blobs = img.find_blobs(
        thresholds,
        x_stride=30,
        y_stride=30,
        invert=False,
        area_threshold=900,
        pixels_threshold=900,
        merge=True)
    if blobs:

        for blob in blobs:
            img.draw_rectangle(blob.rect())

        uart.write("1")
    else:
        uart.write("0")

