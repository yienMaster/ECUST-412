#ifndef __EEPROM_H__
#define __EEPROM_H__

#include "zf_common_headfile.h"

//�洢λ�ñ��
typedef enum
{
    motor_l_p, motor_l_i, motor_l_d,
    motor_r_p, motor_r_i, motor_r_d,
    dir_p, dir_i, dir_d,
    target_speed_p, target_speed_i, target_speed_d,
    para_target_distance_coef,para_dynamic_p_coef,para_target_speed_offset_coef,para_point_distance,
    para_num
}parameter;


#define FLASH_SECTION_INDEX       (0)                                 // �洢�����õ�����
#define FLASH_PAGE_INDEX          (11)                                // �洢�����õ�ҳ�� ������һ��ҳ��


void EEPROM_Update_All_Para(void);                      //�������в�����ֵ
float EEPROM_Read_Data(parameter para);                 //������
void EEPROM_Write_Data(parameter para, float data);     //д����
void Para_Init(void);
void EEPROM_Write_Data_uint16(parameter para, uint16 data);

#endif
