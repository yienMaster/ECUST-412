cmake_minimum_required(VERSION 2.8)
project( main LANGUAGES C CXX)

# 仅查找需要的 OpenCV 组件，排除 gapi
find_package( OpenCV REQUIRED COMPONENTS core videoio imgproc highgui)

add_executable(main main.cpp  servo.c pwm.c gpio.cpp tools.cpp control.cpp motor.c encoder.cpp register.cpp debug.cpp)

# 取消注释并确保路径正确
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#include_directories(/home/szw/car/opencv-4.10.0/build-loongarch64/include/opencv4/opencv2)

# 移除重复的 find_library 调用，使用 find_package 找到的库
# 以下是原有的 find_library 调用，已注释掉
#find_library(OPENCV_CORE opencv_core  /home/szw/car/opencv-4.10.0/build/lib)
#find_library(OPENCV_VIDEOIO opencv_videoio  /home/szw/car/opencv-4.10.0/build/lib)
# find_library(OPENCV_IMGPROC opencv_imgproc  /home/szw/car/opencv-4.10.0/build/lib)
#find_library(OPENCV_HIGHGUI opencv_highgui  /home/szw/car/opencv-4.10.0/build/lib)
#find_library(OPENCV_VIDEOIO opencv_videoio  /home/szw/car/opencv-4.10.0/build/lib)
#find_library(OPENCV_IMGPROC opencv_imgproc /home/szw/car/opencv-4.10.0/build/lib)

# 链接找到的 OpenCV 组件
target_link_libraries(main ${OpenCV_LIBS} )

set(CMAKE_BUILD_TYPE "Debug") # 设置为 Debug 模式



set(CMAKE_SYSTEM_NAME LoongOS)

#/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.4/bin/
set(CMAKE_C_COMPILER loongarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER loongarch64-linux-gnu-g++)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

