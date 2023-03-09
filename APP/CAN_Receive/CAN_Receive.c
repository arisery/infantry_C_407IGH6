/*
 * CAN_Receive.c
 *
 *  Created on: Apr 5, 2022
 *      Author: tl
 */

#include <CAN_Receive.h>
#include "main.h"
#include"function.h"
#include"remote_control.h"
#include "gimbal.h"
#include "Shoot.h"
extern CAN_HandleTypeDef hcan1, hcan2;
static uint16_t can1_cnt = 0;
static uint16_t can2_cnt = 0;
static motor_message_t motor_chassis[4], motor_gimbal[2],motor_shoot[2];
extern gimbal_t gimbal;
extern shoot_t shoot;
void Motor_Message(uint8_t index,motor_message_t* motor,uint8_t *data);
void can_filter_init()
{
	CAN_FilterTypeDef filter_structure;

	filter_structure.FilterIdHigh = 0x0000;
	filter_structure.FilterIdLow = 0x0000;
	filter_structure.FilterMaskIdHigh = 0x0000;
	filter_structure.FilterMaskIdLow = 0x0000;
	filter_structure.FilterMode = CAN_FILTERMODE_IDMASK;
	filter_structure.FilterScale = CAN_FILTERSCALE_32BIT;
	filter_structure.FilterActivation = CAN_FILTER_ENABLE;
	filter_structure.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	filter_structure.FilterBank = 0;
	filter_structure.SlaveStartFilterBank = 14;
	/*********CAN1**************************************/
	HAL_CAN_ConfigFilter(&hcan1, &filter_structure);
	HAL_CAN_Start(&hcan1);
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	/**************CAN2*************************************/
	filter_structure.FilterBank = 14;
	filter_structure.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	HAL_CAN_ConfigFilter(&hcan2, &filter_structure);
	HAL_CAN_Start(&hcan2);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if (hcan == &hcan1)
	{
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

		if ((rx_header.StdId >= FEEDBACK_ID_BASE_3508)
				&& (rx_header.StdId <= FEEDBACK_ID_BASE_3508 + MOTOR_MAX_NUM))
		{
			can1_cnt++;
			uint8_t index = rx_header.StdId - 0x201;
			Motor_Message(index, motor_chassis, rx_data);
			//motor_ecd_to_angle_change(&gimbal.axis[index].motor);
		}
		else if(rx_header.StdId==0x205)
		{
			Motor_Message(0, motor_shoot, rx_data);
			motor_ecd_to_angle_change(&shoot.motor);
		}
#ifdef DEBUG
		if (can1_cnt == 300)
		{
			can1_cnt = 0;
			Toggle_LED_B;

		}
#endif
	}

}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if (hcan == &hcan2)
	{
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_data);

		if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020)
				&& (rx_header.StdId <= FEEDBACK_ID_BASE_6020 + MOTOR_MAX_NUM))
		{
			can2_cnt++;
			uint8_t index = rx_header.StdId - 0x205;
			Motor_Message(index, motor_gimbal, rx_data);
			motor_ecd_to_angle_change(&gimbal.axis[index].motor);
		}
		else if((rx_header.StdId >= FEEDBACK_ID_BASE_2006)
				&& (rx_header.StdId <= FEEDBACK_ID_BASE_6020))
		{
			uint8_t index = rx_header.StdId - 0x201;
			Motor_Message(index, motor_shoot, rx_data);
			//motor_ecd_to_angle_change(&gimbal.axis[index].motor);
		}

		if (can2_cnt == 300)
		{
			can2_cnt = 0;
			//Toggle_LED_R;


		}
	}
}

//发送电机电压*
void set_motor_voltage_CAN1(uint16_t StdId,int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	static CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];

	tx_header.StdId = StdId;//0x200
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	tx_data[0] = (v1 >> 8) & 0xff;
	tx_data[1] = (v1) & 0xff;
	tx_data[2] = (v2 >> 8) & 0xff;
	tx_data[3] = (v2) & 0xff;
	tx_data[4] = (v3 >> 8) & 0xff;
	tx_data[5] = (v3) & 0xff;
	tx_data[6] = (v4 >> 8) & 0xff;
	tx_data[7] = (v4) & 0xff;
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,
			(uint32_t*) CAN_TX_MAILBOX0);

}
//发送电机电压
void set_motor_voltage_CAN2(uint16_t StdId,int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	static CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];

	tx_header.StdId = StdId;
	tx_header.IDE = CAN_ID_STD;
	tx_header.RTR = CAN_RTR_DATA;
	tx_header.DLC = 8;

	tx_data[0] = (v1 >> 8) & 0xff;
	tx_data[1] = (v1) & 0xff;
	tx_data[2] = (v2 >> 8) & 0xff;
	tx_data[3] = (v2) & 0xff;
	tx_data[4] = (v3 >> 8) & 0xff;
	tx_data[5] = (v3) & 0xff;
	tx_data[6] = (v4 >> 8) & 0xff;
	tx_data[7] = (v4) & 0xff;
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data,
			(uint32_t*) CAN_TX_MAILBOX1);

}
//返回底盘电机变量地址，通过指针方式获取原始数据
 motor_message_t* get_Chassis_Motor_Measure_Point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}
 motor_message_t* get_gimbal_Motor_Measure_Point(uint8_t i)
{
	return &motor_gimbal[(i & 0x03)];
}
 motor_message_t* get_shoot_Motor_Measure_Point()
{
	return &motor_shoot[0];
}
void Motor_Message(uint8_t index,motor_message_t* motor,uint8_t *data)
{
	motor[index].last_encoder_value = motor[index].encoder_value;
			motor[index].encoder_value = ((data[0] << 8) | data[1]);
			motor[index].speed_rpm = ((data[2] << 8) | data[3]);
			motor[index].given_current =
					((data[4] << 8) | data[5]);
			motor[index].temperate = data[6];

}
