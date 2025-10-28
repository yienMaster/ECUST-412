import cv2
import socket
import numpy as np
import time  # 新增

import datetime
 
# 获取当前时间


PORT = int(input("port:"))
BUFFER_SIZE = 65536

name = input("name:")


while True:
    # 1. 初始化UDP套接字
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', PORT))

    current_time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    print("current time:", current_time)
    # 2. 初始化视频保存
    fourcc = cv2.VideoWriter_fourcc(*'MJPG')
    out = cv2.VideoWriter(name+'-'+current_time+'.avi', fourcc, 30.0, (160, 120))


    last_recv_time = time.time()
    TIMEOUT = 2  # 超时时间(秒)

    count=0

    # 3. 接收循环
    jpeg_data = bytearray()
    while True:
        try:
            sock.settimeout(TIMEOUT)  # 设置socket超时
            data, _ = sock.recvfrom(BUFFER_SIZE)
            last_recv_time = time.time()  # 重置计时器
            jpeg_data.extend(data)
            
            # 尝试解码完整JPEG帧
            if len(data) < BUFFER_SIZE:  # 检测到可能的分包结束
                try:
                    frame = cv2.imdecode(np.frombuffer(jpeg_data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        cv2.imshow('Receiver', frame)
                        out.write(frame)
                        count+=1
                        if cv2.waitKey(1) == 27:
                            break
            
                except:
                    pass
                jpeg_data = bytearray()  # 重置缓冲区

        except socket.timeout:
        # 超时后判断是否长时间无数据
            if time.time() - last_recv_time > TIMEOUT and count >0:
                print("传输超时，退出接收")
                break
                

    

    out.release()
    cv2.destroyAllWindows()
    sock.close()

