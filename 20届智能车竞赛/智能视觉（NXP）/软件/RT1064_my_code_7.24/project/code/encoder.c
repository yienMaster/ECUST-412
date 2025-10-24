#include "my_code.h"
#include "math.h"

#define MAX_ENCODER_CHANGE 1000

int16 encoder1_data = 0, encoder2_data = 0, encoder3_data = 0;
int16 prev_e1data = 0,prev_e2data = 0, prev_e3data = 0;
bool e1_outOfLimit = false;
bool e2_outOfLimit = false;
bool e3_outOfLimit = false;
void limit_edata()
{
	//encoder1 limit
	if(e1_outOfLimit)
	{
		e1_outOfLimit = false;
		encoder1_data = prev_e1data;
	}
	else 
	if(abs(encoder1_data) > MAX_ENCODER_CHANGE)
	{
		encoder1_data = prev_e1data;
		e1_outOfLimit = true;
	}
	else prev_e1data = encoder1_data;
	
	//encoder2 limit
	if(e2_outOfLimit)
	{
		e2_outOfLimit = false;
		encoder2_data = prev_e2data;
	}
	else 
	if(abs(encoder2_data) > MAX_ENCODER_CHANGE)
	{
		encoder2_data = prev_e2data;
		e2_outOfLimit = true;
	}
	else prev_e2data = encoder2_data;
	
	//encoder3 limit
	if(e3_outOfLimit)
	{
		e3_outOfLimit = false;
		encoder3_data = prev_e3data;
	}
	else 
	if(abs(encoder3_data) > MAX_ENCODER_CHANGE)
	{
		encoder3_data = prev_e3data;
		e3_outOfLimit = true;
	}
	else prev_e3data = encoder3_data;
}
void encoder_init()
{
		encoder_quad_init(ENCODER1_QUADDEC, ENCODER1_QUADDEC_A, ENCODER1_QUADDEC_B);
		encoder_quad_init(ENCODER2_QUADDEC, ENCODER2_QUADDEC_A, ENCODER2_QUADDEC_B);
		encoder_quad_init(ENCODER3_QUADDEC, ENCODER3_QUADDEC_A, ENCODER3_QUADDEC_B);

}
int16 get_encoder1_data()
{
		int16 encoder1_data = encoder_get_count(ENCODER1_QUADDEC);  
    encoder_clear_count(ENCODER1_QUADDEC);
		return encoder1_data;
}

int16 get_encoder2_data()
{
		int16 encoder2_data = encoder_get_count(ENCODER2_QUADDEC);  
    encoder_clear_count(ENCODER2_QUADDEC);
		return encoder2_data;
}

int16 get_encoder3_data()
{
		int16 encoder3_data = encoder_get_count(ENCODER3_QUADDEC);  
    encoder_clear_count(ENCODER3_QUADDEC);
		return encoder3_data;
}



