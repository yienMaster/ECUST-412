#include "adc.h"

//电源检测adc初始化
void power_supply_init(void)
{
    adc_init(ADC_power_supply, ADC_12BIT);
}

//获取电源电压
float voltage_adc(void)
{
    float voltage_power_supply = 0;
    voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply, 60) / 1200 / 22 *222;
    return voltage_power_supply;
}
////获取电源电压
//float voltage_adc(void)
//{
//    float voltage_power_supply = 0;
//    voltage_power_supply = (float)adc_mean_filter_convert(ADC_power_supply, 60) / 1200 / 20 *220;
//    return voltage_power_supply;
//}
