/*
 * task.c
 *
 *  Created on: 2022年12月2日
 *      Author: arisery
 */


#include "../task_init/task.h"
#include "main.h"
#include"remote_control.h"
#include"function.h"
#include"CAN_Receive.h"
#include "chassis.h"
#include "drv_RF24L01.h"
#include "sync.h"
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern chassis_struct_t chassis;
uint32_t tim6cnt;
void task_init()
{
	LED_OFF_ALL();

	uart_dma_init();
	can_filter_init();
	chassis_init(&chassis);
	//sync_init();
	HAL_Delay(1000);

 HAL_TIM_Base_Start_IT(&htim3);
 HAL_TIM_Base_Start(&htim6);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

if(htim==&htim3)
{

	TIM6->CNT=0;
	__HAL_TIM_ENABLE(&htim6);

	chassis_task();
#ifdef nrf_rx

	NRF24L01_Flush_Rx_Fifo();
NRF24L01_Clear_IRQ_Flag( IRQ_ALL);
#else

#endif

#ifdef nrf_rx
	static uint16_t i=0;
	if (i==500)
	{
		Toggle_LED_G;
		i=0;
	}
i++;


#else
	static uint16_t i=0;
	if (i==500)
	{
		Toggle_LED_R;
		i=0;
	}
i++;
#endif
}
//tim6cnt=TIM6->CNT;
}
