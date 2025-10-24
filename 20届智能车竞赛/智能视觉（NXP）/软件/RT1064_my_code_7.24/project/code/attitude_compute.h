#ifndef _ATTITUDE_COMPUTE_H_
#define _ATTITUDE_COMPUTE_H_

#include "zf_common_headfile.h"

#define M_PI        3.1415926f

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} imu_struct;


typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_struct;


typedef struct {
    float pitch;    //俯仰角
    float roll;     //偏航角
    float yaw;       //翻滚角
} euler_struct;


typedef struct {
    float Xdata;
    float Ydata;
    float Zdata;
} gyro_struct;

extern euler_struct eulerAngle;

void gyroOffset_init(void);

float fast_sqrt(float x);

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

void IMU_getValues();

void IMU_getEulerianAngles(void);

#endif
