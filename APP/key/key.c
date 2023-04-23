/*
 * key.c
 *
 *  Created on: Feb 24, 2023
 *      Author: arisery
 */

#include "key.h"
#include "remote_control.h"
#include "cmsis_os.h"
KEY_T btn[1],mouse_L,mouse_R,Key_shift,Key_Q;
extern RC_ctrl_t rc_ctrl;
key_state key_detect(KEY_T *key,int8_t val)
{
	int8_t state;
	switch (key->type)
	{
	case button:
		state = val;
		break;
	case flag:
		state =val;
	}
	key->last_state=key->state;
	if (state == key->TrueVal)//按键按下
	{
		if (key->temp == 0)//如果上一次没有按下
		{
			key->temp = 1;
		}
		else if (key->temp == 1) //如果上一次也按下
		{
			if(key->counter<key->LongPressCounter)
			{
			key->state = key_pressd;//按键的状态为按下
			}
			else if(key->counter>key->LongPressCounter)
			{
				key->state = key_LongPressed;

			}
			key->counter++;
		}


	}
	else
	{
		if (key->temp == 1)
		{
			if(key->state != key_LongPressed)
			{
				key->state = key_ShortClicked;
				key->counter=0;
			}
			else if(key->state == key_LongPressed){
				key->state = key_LongClicked;
				key->counter=0;
			}
		}
		else
		{
			key->state = key_release;
		}
		key->temp =0;

	}
return key->state;
}

void key_init()
{
	btn[0].GPIO.GPIOX = GPIOA;
	btn[0].GPIO.GPIO_PIN = GPIO_PIN_0;
	btn[0].type=button;
	btn[0].TrueVal=GPIO_PIN_RESET;
	btn[0].LongPressCounter=30;


	mouse_L.type=flag;
	mouse_L.TrueVal=1;
	mouse_L.LongPressCounter=30;


	mouse_R.type=flag;
	mouse_R.TrueVal=1;
	mouse_R.LongPressCounter=30;

	Key_shift.type=flag;
	Key_shift.TrueVal=1;
	Key_shift.LongPressCounter=30;
	Key_Q.type=flag;
	Key_Q.TrueVal=1;
	Key_Q.LongPressCounter=30;
}

void Key_Task(void const * argument)
{
	key_init();
	for(;;)
	{
		key_detect(btn, HAL_GPIO_ReadPin(btn[0].GPIO.GPIOX , btn[0].GPIO.GPIO_PIN));
		key_detect(&mouse_L, rc_ctrl.mouse.press_l);
		key_detect(&mouse_R, rc_ctrl.mouse.press_r);
		key_detect(&Key_shift, rc_ctrl.keyboard.key.SHIFT);
		key_detect(&Key_Q, rc_ctrl.keyboard.key.Q);
		osDelay(5);
	}
}

