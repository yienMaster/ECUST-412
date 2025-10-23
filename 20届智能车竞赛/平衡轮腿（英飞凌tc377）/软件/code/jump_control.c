#include "jump_control.h"
#include "engine.h"
#include "control.h"
#include "FiveBarLinkageData.h"
#include "image.h"
extern IMU_t IMU_data;
int jump_stop = 0;
JumpState jump_state = JUMP_FREE;

// 定义跳跃过程中的目标位置
#define PREPARE_X 0.00  // 中间位置
#define PREPARE_Y 0.03

#define BURST_X 0.00   // 中间位置
#define BURST_Y 0.14

#define AIR_RETRACT_X 0.00  // 中间位置
#define AIR_RETRACT_Y 0.03

#define PRE_BUFFER_X 0.00  // 前倾位置
#define PRE_BUFFER_Y 0.05

#define EXE_BUFFER_X -0.01  // 更大前倾
#define EXE_BUFFER_Y 0.05

#define RECOVER_X 0.00  // 恢复中间位置
#define RECOVER_Y 0.04

// 跳跃控制函数
void jump_process_control(float *current_x, float *current_y) {
    int leg1, leg2;
    switch (jump_state) {
        case JUMP_PREPARE:
            // 准备阶段，调整到起始位置
            jump_stop = 0;
            if ( *current_y!= PREPARE_Y) {
                *current_y = PREPARE_Y;
                servo_control(*current_x, *current_y, &leg1, &leg2);//控制当前的x不变，跳跃y
                engine_maintain(leg1, leg2);
                system_delay_ms(50);
                jump_state = JUMP_BURST;
            } else {
                // 准备阶段完成，进入爆发阶段
                jump_state = JUMP_BURST;
            }

        case JUMP_BURST:
            // 爆发阶段，快速伸展
            //*current_x = BURST_X;
           // *current_y = BURST_Y;
            //servo_control(*current_x, *current_y, &leg1, &leg2);
            engine_maintain(1050, 1050);
            jump_state = JUMP_EXE_BUFFER;
            system_delay_ms(130);

//        case JUMP_AIR_RETRACT:
//            // 空中收缩阶段
//            *current_x = AIR_RETRACT_X;
//            *current_y = AIR_RETRACT_Y;
//            servo_control(*current_x, *current_y, &leg1, &leg2);
//            engine_maintain(leg1, leg2);
//            jump_state = JUMP_PRE_BUFFER;
//            system_delay_ms(80);


//        case JUMP_PRE_BUFFER:
//            // 缓冲准备阶段，前倾
//            *current_x = PRE_BUFFER_X;
//            *current_y = PRE_BUFFER_Y;
//            servo_control(*current_x, *current_y, &leg1, &leg2);
//            engine_maintain(leg1, leg2);
//            system_delay_ms(50);
//

        case JUMP_EXE_BUFFER://空中状态
            // 缓冲执行阶段，进一步前倾
            jump_stop = 1;
            *current_x = EXE_BUFFER_X;
            *current_y = EXE_BUFFER_Y;
            servo_control(*current_x, *current_y, &leg1, &leg2);
            engine_maintain(leg1, leg2);
            while(IMU_data.accel[2] < 3.0f) //判断角速度,碰撞加速度
            { // 检测到接触地面
            }
            jump_state = JUMP_RECOVER;
        case JUMP_RECOVER:
            // 恢复阶段，回到中间位置
            jump_stop = 0;
            *current_x = RECOVER_X;
            *current_y = RECOVER_Y;
            servo_control(*current_x, *current_y, &leg1, &leg2);
            engine_maintain(leg1, leg2);
            jump_position = 0;
            system_delay_ms(200);
            jump_state = JUMP_FREE;

        case JUMP_FREE:

            break;

            // 自由状态

    }
}

// 跳跃中止函数
void jump_abort(void) {
    engine_maintain(600, 600);  // 设置伺服电机为中间位置
    jump_state = JUMP_PREPARE;  // 重置跳跃状态为准备状态
}
