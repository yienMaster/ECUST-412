#ifndef JUMP_CONTROL_H
#define JUMP_CONTROL_H


// ��Ծ״̬��
typedef enum {
    JUMP_PREPARE,       // ׼���׶�
    JUMP_BURST,         // �����׶�
    JUMP_AIR_RETRACT,   // ��������
    JUMP_PRE_BUFFER,    // ׼������
    JUMP_EXE_BUFFER,    // ִ�л���
    JUMP_RECOVER  ,     // �ָ��׶�
    JUMP_FREE//����״̬
} JumpState;
extern int jump_stop;
extern JumpState jump_state;
void jump_process_control(float *current_leg_length,float *angle);
void jump_abort(void);

#endif
