#ifndef CODE_ADC_H_
#define CODE_ADC_H_
#include "zf_common_headfile.h"

#define ADC_power_supply              (ADC2_CH4_A20)//��Դ���ADC����
//#define ADC_power_supply              (ADC1_CH3_A11)//��Դ���ADC����

extern float voltage_power_supply;//��Դ��ѹ
void power_supply_init(void);
float voltage_adc(void);

#endif /* CODE_ADC_H_ */
