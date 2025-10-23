#ifndef CODE_TFT_H_
#define CODE_TFT_H_

#include "zf_common_headfile.h"
#include "image.h"
#include "wifi.h"
#include "adc.h"

#define KEY1                    (P33_12)
#define KEY2                    (P32_4)
#define KEY3                    (P20_7)
//#define KEY1                    (P11_3)
//#define KEY2                    (P11_2)
//#define KEY3                    (P20_7)

extern float now_start;//��ʼ���б�־λ
extern float V_a;//�ٶȸ�ֵ

//��ʼ��
void TFT_init(void);
void KEY_init(void);

//�����˵�
void TFT_KEYput(uint8 get_key1,uint8 get_key2,uint8 get_key3);


#endif /* CODE_ADC_H_ */
