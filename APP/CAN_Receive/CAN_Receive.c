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
static motor_message_t motor_chassis[4], motor_gimbal[2], motor_friction[2], motor_supply;
extern gimbal_t gimbal;
extern shoot_t shoot;
static void Motor_Message(uint8_t index, motor_message_t *motor, uint8_t *data);

/*
 * CAN的过滤器初始化
 * 在CAN总线上只有几个大疆的电机，所以不做ID掩码过滤
 */
void CAN_Filter_Init()
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
	//先开启CAN1再开启中断
	HAL_CAN_Start(&hcan1);
	//将CAN1接收的消息指定到FIFO0
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	/**************CAN2*************************************/
	//先开启CAN1才能开启CAN2，因为CAN2的时钟是依托于CAN1的
	filter_structure.FilterBank = 14;
	filter_structure.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	HAL_CAN_ConfigFilter(&hcan2, &filter_structure);
	HAL_CAN_Start(&hcan2);
	//将CAN2接收的消息指定到FIFO1
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}
//FIFO0的接收中断
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if (hcan == &hcan1)
	{
		//从FIFO0中获取数据
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, rx_data);

		if ((rx_header.StdId > FEEDBACK_ID_BASE_3508) && (rx_header.StdId <= FEEDBACK_ID_BASE_3508 + MOTOR_MAX_NUM))
		{
			uint8_t index = rx_header.StdId - 0x201;
			Motor_Message(index, motor_chassis, rx_data);

		}
		else if (rx_header.StdId == 0x205)
		{
			Motor_Message(0, &motor_supply, rx_data);
			motor_ecd_to_angle_change(&shoot.Supply.SupplyMotor);
		}
	}

}
//FIFO1的接收中断
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	if (hcan == &hcan2)
	{
		//从FIFO1中获取数据
		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO1, &rx_header, rx_data);
		//获取云台数据
		if ((rx_header.StdId >= FEEDBACK_ID_BASE_6020) && (rx_header.StdId <= FEEDBACK_ID_BASE_6020 + 2))
		{
			uint8_t index = rx_header.StdId - 0x205;
			Motor_Message(index, motor_gimbal, rx_data);
			motor_ecd_to_angle_change(&gimbal.axis[index].motor);
		}
		//获取摩擦轮数据
		else if ((rx_header.StdId > 0x206) && (rx_header.StdId <= 0x208))
		{
			uint8_t index = rx_header.StdId - 0x207;
			Motor_Message(index, motor_friction, rx_data);

		}
	}
}

/*
 * 通过CAN1发送电机电流数据
 * StdId：标识符
 */
void SetMotorVoltage_CAN1(uint16_t StdId, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
	static CAN_TxHeaderTypeDef tx_header;
	uint8_t tx_data[8];

	tx_header.StdId = StdId; //0x200
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
	HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, (uint32_t*) CAN_TX_MAILBOX0);

}
/*
 * 通过CAN2发送电机电流数据
 * StdId：标识符
 */
void SetMotorVoltage_CAN2(uint16_t StdId, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
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
	HAL_CAN_AddTxMessage(&hcan2, &tx_header, tx_data, (uint32_t*) CAN_TX_MAILBOX1);

}
//返回底盘电机变量地址，通过指针方式获取原始数据
//获取底盘电机信息结构体指针
motor_message_t* Get_ChassisMotor_MessagePoint(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}
//获取云台电机信息结构体指针
motor_message_t* Get_GimbalMotor_MessagePoint(uint8_t i)
{
	return &motor_gimbal[(i & 0x03)];
}
//获取拨弹轮电机信息结构体指针
motor_message_t* Get_SupplyMotor_MessagePoint()
{
	return &motor_supply;

}
//获取摩擦轮电机信息结构体指针
motor_message_t* Get_FrictionMotor_MessagePoint(uint8_t i)
{
	if (i >= 2)
	{
		return 0;
	}
	return &motor_friction[i];

}
/*
 * 对电机传回的数据进行解码
 * 	index：电机的序号
 * 	motor：电机信息结构体地址
 * 	data：需要解码的原始数据
 */
void Motor_Message(uint8_t index, motor_message_t *motor, uint8_t *data)
{
	//将编码器值保存为上一次编码器值
	motor[index].last_encoder_value = motor[index].encoder_value;
	//更新编码器值
	motor[index].encoder_value = ((data[0] << 8) | data[1]);
	//更新转速，单位为RPM
	motor[index].speed_rpm = ((data[2] << 8) | data[3]);
	//更新当前电机电流值
	motor[index].given_current = ((data[4] << 8) | data[5]);
	//更新电机温度
	motor[index].temperate = data[6];

}
