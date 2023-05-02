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
#include "stdlib.h"
#include "remote_control.h"
uint32_t rx_count = 0;
MSG_t MSG;
extern gimbal_t gimbal;
extern RC_ctrl_t rc_ctrl;
Referee_t referee;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart1)
	{

		if (Size == 16)
		{


			switch (MSG.ID)
			{
				case vision_e:
					if((abs(MSG.array[0])<320)&&(abs(MSG.array[1])<256))
		{
					gimbal.vision.x_set = MSG.array[0];
					gimbal.vision.y_set = MSG.array[1];
					gimbal.vision.OpenFire= MSG.array[2];
					rc_ctrl.mouse.press_l=MSG.array[2];
					rx_count++;
		}
					break;
				case move_e:

					break;
				case referee_e:
					if((MSG.array[0]>=0)&&(MSG.array[0]<400)&&( MSG.array[6]>0)&&( MSG.array[6]<4))
					{
					referee.blood = MSG.array[0];
					referee.watt.watt_i = MSG.array[2] << 16 | MSG.array[1];
					referee.joule = MSG.array[3];
					referee.bullet_speed.bullet_speed_i = MSG.array[4] << 16 | MSG.array[5];
					referee.level = MSG.array[6];
					}
					break;
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t*) &MSG, 32);
	}
}

void vision_RX_init()
{
	//115200
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*) &MSG, 32);

}
