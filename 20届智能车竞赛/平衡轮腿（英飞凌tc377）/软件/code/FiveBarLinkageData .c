#include <stdio.h>
#include <math.h>
#include "FiveBarLinkageData.h"

// 瀹氫箟 x 鍜� y 鐨勮寖鍥村強姝ラ暱
#define MIN_X -0.05
#define MAX_X 0.05
#define X_STEP 0.001

#define MIN_Y 0.02
#define MAX_Y 0.14
#define Y_STEP 0.001
//==================================================���� �ȳ� ��������================================================
#define L1  0.06
#define L2  0.09
#define L3  0.09
#define L4  0.06
#define L5  0.038
#define PI 3.141592653589793
// 鑾峰彇鍏宠妭瑙掑害
void getJointAngles(float x_target, float y_target, float *phi1, float *phi4) {
    float a = 2 * (x_target + L5 / 2) * L1;
        float b = 2 * y_target * L1;
        float c = pow((x_target + L5 / 2), 2) + pow(y_target, 2) + pow(L1, 2) - pow(L2, 2);

        float sum_val = pow(a, 2) + pow(b, 2) - pow(c, 2);

        if (sum_val >= 0) {
            float phi1_rad = 2 * atan((b + sqrt(sum_val)) / (a + c)); // 只取正解
            *phi1 = phi1_rad * (180.0 / PI); // 弧度转角度

            if (*phi1 > 360) {
                *phi1 -= 360;
            } else if (*phi1 < 0) {
                *phi1 += 360;
            }
        } else {
            *phi1 = 400; // 无解
        }

        // 计算第二个关节角度 phi4
        float a1 = 2 * (x_target - L5 / 2) * L4; // 这里应该是 L4 而不是 L1
        float b1 = 2 * y_target * L4;
        float c1 = pow((x_target - L5 / 2), 2) + pow(y_target, 2) + pow(L4, 2) - pow(L3, 2);

        sum_val = pow(a1, 2) + pow(b1, 2) - pow(c1, 2);

        if (sum_val >= 0) {
            float phi4_rad = 2 * atan((b1 - sqrt(sum_val)) / (a1 + c1)); // 只取负解
            *phi4 = phi4_rad * (180.0 / PI); // 弧度转角度

            if (*phi4 > 360) {
                *phi4 -= 360;
            } else if (*phi4 < 0) {
                *phi4 += 360;
            }
        } else {
            *phi4 = 400; // 无解
        }
}

// 浼烘湇鐢垫満鎺у埗
void servo_control(float x, float y, int *leg1, int *leg2) {
    float phi1, phi4;

    // 鑾峰彇鍏宠妭瑙掑害
    getJointAngles(x, y, &phi1, &phi4);
        if(phi1==400||phi4==400)
        {

        }
        else
        {
             if((phi1>=99&&phi1<=261)&&((phi4 >= 279)||(phi4 <= 81)))
            {
                *leg1 = (270 - phi1) / 180 * 1000 + 250;
                if (phi4 >= 270)
                {
                    *leg2 = (int)((phi4 - 270) / 180 * 1000 + 250);
                }
                else if(phi4 <= 90)
                {
                    *leg2 = (int)((90 + phi4) / 180 * 1000 + 250);
                }
            }
            else
            {
                *leg1=*leg2;
                *leg2=*leg2;
            }
        }
 }

