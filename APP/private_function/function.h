/*
 * function.h
 *
 *  Created on: Apr 19, 2022
 *      Author: tl
 */

#ifndef PRIVATE_FUNCTION_FUNCTION_H_
#define PRIVATE_FUNCTION_FUNCTION_H_
#include "main.h"





/********************LED********************/
//Toggle LED
#define Toggle_LED_R  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
#define Toggle_LED_G  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
#define Toggle_LED_B  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
//LED ON
#define LED_R_ON HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,SET);
#define LED_G_ON HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,SET);
#define LED_B_ON HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,SET);
//LED OFF
#define LED_R_OFF HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin,RESET);
#define LED_G_OFF HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin,RESET);
#define LED_B_OFF HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin,RESET);


void LED_ON_ALL(void);
void LED_OFF_ALL(void);

void buzzer_off(void);
void buzzer_on(int pwm);

#endif /* PRIVATE_FUNCTION_FUNCTION_H_ */
