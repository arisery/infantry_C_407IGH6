/*
 * vision.c
 *
 *  Created on: 2023年4月22日
 *      Author: arisery
 */


#include "vision.h"
#include "main.h"
#include "usart.h"
int16_t vision_rx[32], x_p,ID;
vision_t vision;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart6)
	{
		if ((vision_rx[0] == 0x22) && (vision_rx[10] == 0x33))
		{
 vision.header=vision_rx[0];
 vision.tail=vision_rx[10];
 vision.ID=vision_rx[1];
 vision.array[0]=vision_rx[2]<<8|vision_rx[3];
 vision.array[1]=vision_rx[4]<<8|vision_rx[5];
 vision.array[2]=vision_rx[6]<<8|vision_rx[7];
 vision.array[3]=vision_rx[8]<<8|vision_rx[9];
		}
		//printf("X:%d,\tY:%d\tDis:%d\r\n", vision_rx[1], vision_rx[2], vision_rx[3]);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &vision_rx, 32);
	}
}

void vision_RX_init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &vision_rx, 32);

}
