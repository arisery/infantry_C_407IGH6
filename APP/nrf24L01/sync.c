/*
 * sync.c
 *
 *  Created on: Dec 8, 2022
 *      Author: arisery
 */


#include "sync.h"
#include "drv_RF24L01.h"
#include "main.h"
#include "stdio.h"
#include"function.h"
#include "remote_control.h"
uint8_t nrf_buff[36]={"abcd,abcd,abcd,abcd,abcd,abcd"},nrf_rx_buff[36];
extern UART_HandleTypeDef huart1;
extern RC_ctrl_t rc_ctrl;
void sync_init()
{

	/****
	 *DEBUG版作为接收端
	 */
#ifdef nrf_rx
	init_rx();

	/*
	 * release版作为发送端
	 */

#else
	init_tx();
#endif

}
void read_reg() {
	printf("config:%d\r\n", NRF24L01_Read_Reg(CONFIG));
	printf("ENAA:%d\r\n", NRF24L01_Read_Reg(EN_AA));
	printf("EN_RXADDR:%d\r\n", NRF24L01_Read_Reg(EN_RXADDR));
	printf("SETUP_AW:%d\r\n", NRF24L01_Read_Reg(SETUP_AW));
	printf("SETUP_RETR:%d\r\n", NRF24L01_Read_Reg(SETUP_RETR));
	printf("RF_CH:%d\r\n", NRF24L01_Read_Reg(RF_CH));
	printf("RF_SETUP:%d\r\n", NRF24L01_Read_Reg(RF_SETUP));
	printf("STATUS:%d\r\n", NRF24L01_Read_Reg(STATUS));
	printf("OBSERVE_TX:%d\r\n", NRF24L01_Read_Reg(OBSERVE_TX));
	printf("RSSI:%d\r\n", NRF24L01_Read_Reg(RSSI));
	//read_string(RX_ADDR_P0, (uint8_t*) &addr, 5);
	//printf("RX_ADDR_P0:%s\r\n", addr);
	printf("RX_PW_P0:%d\r\n", NRF24L01_Read_Reg(RX_PW_P0));
	printf("NRF_FIFO_STATUS:%d\r\n", NRF24L01_Read_Reg(NRF_FIFO_STATUS));
	printf("FEATRUE:%d\r\n", NRF24L01_Read_Reg(FEATRUE));
	/*
	config:10
	ENAA:1
	EN_RXADDR:1
	SETUP_AW:3
	SETUP_RETR:255
	RF_CH:60
	RF_SETUP:38
	STATUS:46
	OBSERVE_TX:0
	RSSI:0
	RX_ADDR_P0:
	RX_PW_P0:5
	NRF_FIFO_STATUS:17
	FEATRUE:1
	*/
}
#ifdef nrf_rx

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	 uint16_t irq_flag=0,lenth;
		Toggle_LED_R;
	if(GPIO_Pin==IRQ_Pin)
	{


		irq_flag = NRF24L01_Read_Reg( STATUS);		//读状态寄存器
		//清中断标�??????????????
		if (irq_flag & RX_OK)	//接收到数�??????????????
		{
			NRF24L01_Write_Reg( STATUS, irq_flag);

			lenth = NRF24L01_Read_Rx_Payload((uint8_t*)&nrf_rx_buff);
			printf("lenth:%d,irq:%d",lenth,irq_flag);
NRF24L01_Flush_Rx_Fifo();
NRF24L01_Clear_IRQ_Flag( IRQ_ALL);
SBUS_TO_RC((uint8_t*)&nrf_rx_buff, &rc_ctrl);
//HAL_UART_Transmit(&huart1, (uint8_t*)&nrf_rx_buff, 18, 0xfff);
		}

		NRF24L01_Flush_Rx_Fifo();
 NRF24L01_Clear_IRQ_Flag( IRQ_ALL);

	}



}

#endif
