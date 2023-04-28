/*
 * vision.c
 *
 *  Created on: 2023年4月22日
 *      Author: arisery
 */

#include "vision.h"
#include "main.h"
#include "usart.h"
#include "gimbal.h"
uint32_t rx_count = 0;
MSG_t MSG;
extern gimbal_t gimbal;
Referee_t referee;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart6)
	{
//		if ((vision_rx[0] == 0x22) && (vision_rx[10] == 0x33))
//		{
// vision.header=vision_rx[0];
// vision.tail=vision_rx[10];
// vision.ID=vision_rx[1];
// vision.array[0]=vision_rx[3]<<8|vision_rx[2];
// vision.array[1]=vision_rx[5]<<8|vision_rx[4];
// vision.array[2]=vision_rx[7]<<8|vision_rx[6];
//	vision.array[3]=vision_rx[9]<<8|vision_rx[8];
//		}
		//printf("X:%d,\tY:%d\tDis:%d\r\n", vision_rx[1], vision_rx[2], vision_rx[3]);

		if (Size == 18)
		{
			switch (MSG.ID)
			{
				case vision_e:
					gimbal.vision.x_set = MSG.array[0];
					gimbal.vision.y_set = MSG.array[1];
					rx_count++;
					break;
				case move_e:

					break;
				case referee_e:
					referee.blood = MSG.array[0];
					referee.watt.watt_i = MSG.array[2] << 16 | MSG.array[1];
					referee.joule = MSG.array[3];
					referee.bullet_speed.bullet_speed_i = MSG.array[4] << 16 | MSG.array[5];
					referee.level = MSG.array[6];
					break;
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &MSG, 32);
	}
}

void vision_RX_init()
{
	//115200
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &MSG, 32);

}
