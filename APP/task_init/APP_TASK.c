/*
 * task.c
 *
 *  Created on: 2022年12月2日
 *      Author: arisery
 */


#include <APP_TASK.h>
#include <chassis.h>
#include <CAN_Receive.h>
#include "main.h"
#include"remote_control.h"
#include"function.h"
#include "drv_RF24L01.h"
#include "sync.h"
#include "gimbal.h"
#include "Shoot.h"
#include "vision.h"
extern TIM_HandleTypeDef htim3,htim2;
extern TIM_HandleTypeDef htim6;
extern chassis_struct_t chassis;
uint32_t tim6cnt;
void task_init()
{
	LED_OFF_ALL();
	UART_IDLE_DMA_Init();
	CAN_Filter_Init();
	HAL_Delay(2000);
	vision_RX_init();

}

