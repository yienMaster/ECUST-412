#include "eeprom.h"

void EEPROM_Update_All_Para(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    motor_l.p=flash_union_buffer[speed_para_set*para_num+motor_l_p].float_type;
    motor_l.i=flash_union_buffer[speed_para_set*para_num+motor_l_i].float_type;
    motor_l.d=flash_union_buffer[speed_para_set*para_num+motor_l_d].float_type;
    motor_r.p=flash_union_buffer[speed_para_set*para_num+motor_r_p].float_type;
    motor_r.i=flash_union_buffer[speed_para_set*para_num+motor_r_i].float_type;
    motor_r.d=flash_union_buffer[speed_para_set*para_num+motor_r_d].float_type;
    dir.p=flash_union_buffer[speed_para_set*para_num+dir_p].float_type;
    dir.i=flash_union_buffer[speed_para_set*para_num+dir_i].float_type;
    dir.d=flash_union_buffer[speed_para_set*para_num+dir_d].float_type;
    dir_p_set=dir.p;
    target_speed_pid.p=flash_union_buffer[speed_para_set*para_num+target_speed_p].float_type;
    target_speed_pid.i=flash_union_buffer[speed_para_set*para_num+target_speed_i].float_type;
    target_speed_pid.d=flash_union_buffer[speed_para_set*para_num+target_speed_d].float_type;
    target_distance_coef=flash_union_buffer[speed_para_set*para_num+para_target_distance_coef].float_type;
    dynamic_p_coef=flash_union_buffer[speed_para_set*para_num+para_dynamic_p_coef].float_type;
    target_speed_offset_coef=flash_union_buffer[speed_para_set*para_num+para_target_speed_offset_coef].float_type;
    target_point_distance=flash_union_buffer[speed_para_set*para_num+para_point_distance].uint8_type;
}

float EEPROM_Read_Data(parameter para)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    return flash_union_buffer[para].float_type;
}

void EEPROM_Write_Data(parameter para, float data)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_union_buffer[para].float_type=data;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

void EEPROM_Write_Data_uint16(parameter para, uint16 data)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
    flash_union_buffer[para].uint16_type=data;
    flash_write_page_from_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
}

void Para_Init(void)
{
    EEPROM_Write_Data(motor_l_p,59.469);    //19.469
    EEPROM_Write_Data(motor_l_i,2.804); //0.304
    EEPROM_Write_Data(motor_r_p,59.710);    //19.710
    EEPROM_Write_Data(motor_r_i,2.808); //0.308
//    EEPROM_Write_Data(target_speed_p,15);
//    EEPROM_Write_Data(target_speed_i,0.03);
    para_update_flag=1;
    para_update_display_flag=1;
}


