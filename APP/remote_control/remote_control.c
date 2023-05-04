/*
 * remote_control.c
 *
 *  Created on: Apr 1, 2022
 *      Author: tl
 */
#include"remote_control.h"
#include "stm32f4xx.h"
#include "stdio.h"
#include"main.h"
#include"function.h"
#include "drv_RF24L01.h"
#include "sync.h"
#include "math.h"
extern char nrf_buff[36];
extern uint32_t tim6cnt;

#define RC_CHANNAL_ERROR_VALUE 700
extern UART_HandleTypeDef uart_DBUS;
extern DMA_HandleTypeDef hdma_rx_DBUS;
extern float sense_x;
uint8_t SBUS_rx_buf[SBUS_RX_BUF_NUM] = { 0 };
uint16_t this_time_rx_len = 0;
RC_ctrl_t rc_ctrl;
float test_x = 0;

void UART_IDLE_DMA_Init()
{

	/* 	HAL_DMAEx_MultiBufferStart(&hdma_rx_DBUS, (uint32_t) &uart_DBUS.Instance->DR,
	 (uint32_t) &SBUS_rx_buf[0], (uint32_t) &SBUS_rx_buf[1], 36);
	 */
	__HAL_UART_ENABLE_IT(&uart_DBUS, UART_IT_IDLE);
	__HAL_DMA_ENABLE(&hdma_rx_DBUS);
	HAL_UART_Receive_DMA(&uart_DBUS, (uint8_t*) &SBUS_rx_buf[0], 36);

}
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{


	if (huart == &uart_DBUS)
	{

		/*
		 if ((hdma_rx_DBUS.Instance->CR & DMA_SxCR_CT) != 0)
		 {
		 __HAL_DMA_DISABLE(&hdma_rx_DBUS);

		 this_time_rx_len = 36 - __HAL_DMA_GET_COUNTER(&hdma_rx_DBUS);
		 __HAL_DMA_SET_COUNTER(&hdma_rx_DBUS, 36);
		 hdma_rx_DBUS.Instance->CR &= ~(DMA_SxCR_CT);

		 if (this_time_rx_len == 18)
		 {
		 SBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
		 }
		 __HAL_DMA_ENABLE(&hdma_rx_DBUS);

		 }
		 */

		//__HAL_DMA_DISABLE(&hdma_rx_DBUS);
		HAL_UART_DMAStop(huart);
		this_time_rx_len = 36 - __HAL_DMA_GET_COUNTER(&hdma_rx_DBUS);
		__HAL_DMA_SET_COUNTER(&hdma_rx_DBUS, 36);
		//hdma_rx_DBUS.Instance->CR |= DMA_SxCR_CT;
		// __HAL_DMA_ENABLE(&hdma_rx_DBUS);
		//printf("len:%d\r\n", this_time_rx_len);

		if (this_time_rx_len == 18)
		{
			SBUS_TO_RC(&SBUS_rx_buf[0], &rc_ctrl);

		}

		//UART_Start_Receive_DMA(&uart_DBUS,(uint8_t*)&SBUS_rx_buf[0][0],36);

		__HAL_UART_CLEAR_IDLEFLAG(&uart_DBUS);
		HAL_UART_Receive_DMA(huart, (uint8_t*) &SBUS_rx_buf[0], 36);

	}
	/*
	 #ifdef nrf_rx

	 #else
	 NRF24L01_Write_Tx_Payload_NoAck(SBUS_rx_buf, 18);
	 #endif
	 */
}
/*
 * 对传回的数据进行解码，转换成能够使用的数据
 *
 */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	if (sbus_buf == NULL || rc_ctrl == NULL)
	{
		return;
	}

	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff; //!< Channel 0
	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | //!< Channel 2
			(sbus_buf[4] << 10)) & 0x07ff;
	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);           //!< Switch left
	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;     //!< Switch right
	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);       //!< Mouse X axis
	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);       //!< Mouse Y axis
	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);     //!< Mouse Z axis
	rc_ctrl->mouse.press_l = sbus_buf[12];            //!< Mouse Left Is Press ?
	rc_ctrl->mouse.press_r = sbus_buf[13];           //!< Mouse Right Is Press ?
	rc_ctrl->keyboard.value = sbus_buf[14] | (sbus_buf[15] << 8);     //!< KeyBoard value
	rc_ctrl->rc.ch[4] = ((int16_t) sbus_buf[16] | ((int16_t) sbus_buf[17] << 8)) & 0x07FF;
	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
	RC_data_is_error();
	if (rc_ctrl->mouse.x > 2000)
	{
		rc_ctrl->mouse.x = 0;
	}
	if (rc_ctrl->rc.ch[0] > 300)
	{
		LED_R_ON
		;
	}
	else
	{

	}
	//测试用，对比无和有一阶滤波的遥控器值的变化情况
	test_x -= rc_ctrl->mouse.x * sense_x;
}
uint8_t RC_data_is_error(void)
{
	//使用了go to语句 方便出错统一处理遥控器变量数据归零
	if (fabs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (fabs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (fabs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (fabs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
	{
		goto error;
	}
	if (rc_ctrl.rc.s[0] == 0)
	{
		goto error;
	}
	if (rc_ctrl.rc.s[1] == 0)
	{
		goto error;
	}
	return 0;

	error:
	rc_ctrl.rc.ch[0] = 0;
	rc_ctrl.rc.ch[1] = 0;
	rc_ctrl.rc.ch[2] = 0;
	rc_ctrl.rc.ch[3] = 0;
	rc_ctrl.rc.ch[4] = 0;
	//rc_ctrl.rc.s[0] = RC_SW_DOWN;
	//rc_ctrl.rc.s[1] = RC_SW_DOWN;
	rc_ctrl.mouse.x = 0;
	rc_ctrl.mouse.y = 0;
	rc_ctrl.mouse.z = 0;
	rc_ctrl.mouse.press_l = 0;
	rc_ctrl.mouse.press_r = 0;
	rc_ctrl.keyboard.value = 0;
	return 1;
}

const RC_ctrl_t* Get_RemoteControl_Point(void)
{
	return &rc_ctrl;
}
