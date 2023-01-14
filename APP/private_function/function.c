/*
 * function.c
 *
 *  Created on: Apr 19, 2022
 *      Author: tl
 */
#include"function.h"
#include"main.h"
#include"stm32f4xx.h"
#include "tim.h"
extern TIM_HandleTypeDef htim12;
void LED_ON_ALL(void)
{
	LED_G_ON;
	LED_R_ON;

}
void LED_OFF_ALL(void)
{
	LED_G_OFF;
	LED_R_OFF;

}

void buzzer_on( int pwm)
{
	htim4.Instance->ARR=(int)(100000/pwm);
	htim4.Instance->CCR3=(int)(htim4.Instance->ARR/2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
}
void buzzer_off(void)
{
    TIM4->CCR3=0;
}
