#include "leg_adaptive.h"
float leg_error ;
extern IMU_t IMU_data;
float roll_limitation(float leg_error) {
    if (fabs(leg_error) > 0.02)
        return 0.02;
    else if (fabs(leg_error) < 0.001)
        return 0;
    else
        return leg_error;
}

float high_limitation(float leg_target) {
    if (leg_target <= 0.03)
        return 0.03;
    else if (leg_target >= 0.1)
        return 0.1;
    else
        return leg_target;
}

void roll_control(float leg_target, float leg_error) {
    int leg1, leg2;
    int leg3, leg4;
    static int leg1_Last, leg2_Last;
    static int leg3_Last, leg4_Last;
    float left_y ;
    float right_y;
   // leg_error = roll_limitation(leg_error);

    // 左边减少腿高

    left_y = high_limitation(leg_target + leg_error);
    // 右边增加腿高
    right_y = high_limitation(leg_target - leg_error);
    servo_control(0.0, left_y, &leg1, &leg2);
    servo_control(0.0, right_y, &leg3, &leg4);
    engine_left_maintain(leg1, leg2);
    engine_right_maintain(leg3, leg4);
    // 平滑处理
       leg1 = 0.2 * leg1_Last + 0.8 * leg1;
       leg2 = 0.2 * leg2_Last + 0.8 * leg2;
       leg3 = 0.2 * leg3_Last + 0.8 * leg3;
       leg4 = 0.2 * leg4_Last + 0.8 * leg4;
    leg1_Last = leg1;
    leg2_Last = leg2;
    leg3_Last = leg3;
    leg4_Last = leg4;
}

void leg_roll_high(float leg_target, float angle_error) {
    float high = tan((-angle_error) * PI / 180) * 0.042;
    roll_control(leg_target, high);
}

void leg_roll_control(float leg_target, float angle_error)
{
    //左右控制
       static float leg_error_last;
       static float angle_error_last;
       angle_error = 0.2*angle_error_last + 0.8*IMU_data.filter_result.pitch;
       float error=(-0.6-angle_error);
//       if(func_abs(error)<0.2)
//       {
//          error=0;
//       }
       leg_error +=0.0000187*error;
       if(leg_error<-0.025)
       {
          leg_error=-0.025;
       }
       if(leg_error>0.025)
         {
            leg_error=0.025;
         }
       leg_error_last=leg_error;
       angle_error_last = angle_error;
    roll_control(leg_target, -leg_error);

}
