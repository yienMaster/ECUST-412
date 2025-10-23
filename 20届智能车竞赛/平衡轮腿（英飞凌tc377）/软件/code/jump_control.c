#include "jump_control.h"
#include "engine.h"
#include "control.h"
#include "FiveBarLinkageData.h"
#include "image.h"
extern IMU_t IMU_data;
int jump_stop = 0;
JumpState jump_state = JUMP_FREE;

// ������Ծ�����е�Ŀ��λ��
#define PREPARE_X 0.00  // �м�λ��
#define PREPARE_Y 0.03

#define BURST_X 0.00   // �м�λ��
#define BURST_Y 0.14

#define AIR_RETRACT_X 0.00  // �м�λ��
#define AIR_RETRACT_Y 0.03

#define PRE_BUFFER_X 0.00  // ǰ��λ��
#define PRE_BUFFER_Y 0.05

#define EXE_BUFFER_X -0.01  // ����ǰ��
#define EXE_BUFFER_Y 0.05

#define RECOVER_X 0.00  // �ָ��м�λ��
#define RECOVER_Y 0.04

// ��Ծ���ƺ���
void jump_process_control(float *current_x, float *current_y) {
    int leg1, leg2;
    switch (jump_state) {
        case JUMP_PREPARE:
            // ׼���׶Σ���������ʼλ��
            jump_stop = 0;
            if ( *current_y!= PREPARE_Y) {
                *current_y = PREPARE_Y;
                servo_control(*current_x, *current_y, &leg1, &leg2);//���Ƶ�ǰ��x���䣬��Ծy
                engine_maintain(leg1, leg2);
                system_delay_ms(50);
                jump_state = JUMP_BURST;
            } else {
                // ׼���׶���ɣ����뱬���׶�
                jump_state = JUMP_BURST;
            }

        case JUMP_BURST:
            // �����׶Σ�������չ
            //*current_x = BURST_X;
           // *current_y = BURST_Y;
            //servo_control(*current_x, *current_y, &leg1, &leg2);
            engine_maintain(1050, 1050);
            jump_state = JUMP_EXE_BUFFER;
            system_delay_ms(130);

//        case JUMP_AIR_RETRACT:
//            // ���������׶�
//            *current_x = AIR_RETRACT_X;
//            *current_y = AIR_RETRACT_Y;
//            servo_control(*current_x, *current_y, &leg1, &leg2);
//            engine_maintain(leg1, leg2);
//            jump_state = JUMP_PRE_BUFFER;
//            system_delay_ms(80);


//        case JUMP_PRE_BUFFER:
//            // ����׼���׶Σ�ǰ��
//            *current_x = PRE_BUFFER_X;
//            *current_y = PRE_BUFFER_Y;
//            servo_control(*current_x, *current_y, &leg1, &leg2);
//            engine_maintain(leg1, leg2);
//            system_delay_ms(50);
//

        case JUMP_EXE_BUFFER://����״̬
            // ����ִ�н׶Σ���һ��ǰ��
            jump_stop = 1;
            *current_x = EXE_BUFFER_X;
            *current_y = EXE_BUFFER_Y;
            servo_control(*current_x, *current_y, &leg1, &leg2);
            engine_maintain(leg1, leg2);
            while(IMU_data.accel[2] < 3.0f) //�жϽ��ٶ�,��ײ���ٶ�
            { // ��⵽�Ӵ�����
            }
            jump_state = JUMP_RECOVER;
        case JUMP_RECOVER:
            // �ָ��׶Σ��ص��м�λ��
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

            // ����״̬

    }
}

// ��Ծ��ֹ����
void jump_abort(void) {
    engine_maintain(600, 600);  // �����ŷ����Ϊ�м�λ��
    jump_state = JUMP_PREPARE;  // ������Ծ״̬Ϊ׼��״̬
}
