#include "my_code.h"

bool key1_short = false;

void key1_pressed();void key2_pressed();void key3_pressed();void key4_pressed();

key_index_enum key_index_array[KEY_NUMBER] = {KEY_1,KEY_2,KEY_3,KEY_4};

void key_detect()
{
	key_scanner();
	key1_pressed();
	key2_pressed();
	key3_pressed();
	key4_pressed();
}
void key1_pressed()
{
	if( KEY_SHORT_PRESS == key_get_state(KEY_1) )
	{
		if(key1_short) key1_short = false;
		else key1_short = true;
		key_clear_state(KEY_1);
	}
	else
	if( KEY_LONG_PRESS == key_get_state(KEY_1) )
	{
		
		key_clear_state(KEY_1);
	}
}

void key2_pressed()
{
	if( KEY_SHORT_PRESS == key_get_state(KEY_2) )
	{
		tft180_clear();
		key_clear_state(KEY_2);
	}
	else
	if( KEY_LONG_PRESS == key_get_state(KEY_2) )
	{
		
		key_clear_state(KEY_2);
	}
}
void key3_pressed()
{
	if( KEY_SHORT_PRESS == key_get_state(KEY_3) )
	{
		
		key_clear_state(KEY_3);
	}
	else
	if( KEY_LONG_PRESS == key_get_state(KEY_3) )
	{
		
		key_clear_state(KEY_3);
	}
}
void key4_pressed()
{
	if( KEY_SHORT_PRESS == key_get_state(KEY_4) )
	{
		
		key_clear_state(KEY_4);
	}
	else
	if( KEY_LONG_PRESS == key_get_state(KEY_4) )
	{
		
		key_clear_state(KEY_4);
	}
}