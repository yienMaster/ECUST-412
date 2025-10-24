#ifndef _ENCODER_H_
#define _ENCODER_H_

extern int16 encoder1_data;
extern int16 encoder2_data;
extern int16 encoder3_data;

void encoder_init();
void limit_edata();
int16 get_encoder1_data();
int16 get_encoder2_data();
int16 get_encoder3_data();


#endif
