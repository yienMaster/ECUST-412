#include "adc.h"

//��Դ���adc��ʼ��
void power_supply_init(void)
{
    adc_init(ADC_power_supply, ADC_12BIT);
}

//��ȡ��Դ��ѹ
float voltage_adc(void)
{
    float voltage_power_supply = 0;
    voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply, 60) / 1200 / 22 *222;
    return voltage_power_supply;
}
////��ȡ��Դ��ѹ
//float voltage_adc(void)
//{
//    float voltage_power_supply = 0;
//    voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply, 60) / 1200 / 20 *220;
//    return voltage_power_supply;
//}
