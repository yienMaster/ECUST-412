#ifndef _Kalman_H
#define _Kalman_H
typedef struct Kalman_Filter{
    float x_last;
    float P_now;
    float P_last;
    float K;
    float R_cov;
    float Q_cov;
}KF_Struct;

#endif
