/*
 * key.h
 *
 *  Created on: Feb 24, 2023
 *      Author: arisery
 */

#ifndef KEY_H_
#define KEY_H_
#include "main.h"

//短按的状态变化为 release（释放）--pressed(按住）--shortclicked（短按松开）--release（释放）
//长按的状态变化为 release（释放）--pressed(按住）--longpressed(长按中）--longclicked(长按松开）--release（释放）
typedef enum
{

	key_ShortClicked, //短点击
	key_LongClicked,	//长点击
	key_pressd,  //按下中
	key_LongPressed,		//长按中
	key_release		//释放


} key_state;
typedef enum
{
	button,	//实体按键
	flag	//状态标志位
} KeyType;

typedef struct
{
	struct
	{
		GPIO_TypeDef *GPIOX;
		uint16_t GPIO_PIN;
	} GPIO;
	int8_t TrueVal;
	int8_t val;

	KeyType type;
	key_state state;
	uint8_t LongPressCounter;
	uint32_t counter;
	uint8_t temp;
	void *click_isr_callback;

} KEY_T;
void key_init();
key_state key_detect(KEY_T *key,int8_t val);
void Key_Task(void const * argument);
#endif /* KEY_H_ */
