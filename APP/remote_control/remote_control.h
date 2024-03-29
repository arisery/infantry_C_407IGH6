/*
 * remote_control.h
 *
 *  Created on: Apr 1, 2022
 *      Author: tl
 */

#ifndef APP_REMOTE_CONTROL_REMOTE_CONTROL_H_
#define APP_REMOTE_CONTROL_REMOTE_CONTROL_H_
#include "main.h"
#include "stdint.h"
#define uart_DBUS  huart3
#define hdma_rx_DBUS	hdma_usart3_rx
#define uart_user huart1

#define SBUS_RX_BUF_NUM 36u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

typedef struct
{
	struct
	{
		int16_t ch[5];
		char s[2];
	} rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	} mouse;

	union
	{
		uint16_t value;
		struct
		{
			uint16_t W :1;
			uint16_t S :1;
			uint16_t A :1;
			uint16_t D :1;
			uint16_t SHIFT :1;
			uint16_t CTRL :1;
			uint16_t Q :1;
			uint16_t E :1;
			uint16_t R :1;
			uint16_t F :1;
			uint16_t G :1;
			uint16_t Z :1;
			uint16_t X :1;
			uint16_t C :1;
			uint16_t V :1;
			uint16_t B :1;
		} key;
	} keyboard;

} RC_ctrl_t;
void UART_IDLE_DMA_Init();
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart);
extern const RC_ctrl_t* Get_RemoteControl_Point(void);
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);
uint8_t RC_data_is_error(void);
#endif /* APP_REMOTE_CONTROL_REMOTE_CONTROL_H_ */
