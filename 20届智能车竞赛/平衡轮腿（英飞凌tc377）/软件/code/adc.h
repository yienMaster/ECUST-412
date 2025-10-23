#ifndef CODE_ADC_H_
#define CODE_ADC_H_
#include "zf_common_headfile.h"

#define ADC_power_supply              (ADC2_CH4_A20)//电源检测ADC引脚
//#define ADC_power_supply              (ADC1_CH3_A11)//电源检测ADC引脚

extern float voltage_power_supply;//电源电压
void power_supply_init(void);
float voltage_adc(void);

#endif /* CODE_ADC_H_ */
