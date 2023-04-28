/*
 * vision.h
 *
 *  Created on: 2023年4月22日
 *      Author: arisery
 */

#ifndef VISION_H_
#define VISION_H_
#include "main.h"

typedef enum
{
	vision_e = 0x1A, //视觉云台控制
	move_e = 0x2A, //哨兵移动
	referee_e = 0x3A //裁判系统

} MSG_ID;
typedef struct
{
	uint8_t header;
	/*
	 * 视觉 0x1A
	 * 哨兵移动0x2A
	 * 裁判系统数据0x3A
	 */
	MSG_ID ID;
	int16_t array[7];
	uint8_t tail;
} MSG_t;

typedef struct
{
	int16_t x_set;
	int16_t y_set;
} vision_t;
typedef struct
{
	//血量
	int16_t blood;

	//功率
	union
	{
		float watt_f;
		int32_t watt_i;
	} watt;
	//缓冲
	int16_t joule;

	//弹速
	union
	{
		int16_t bullet_speed_i;
		float bullet_speed_f;
	} bullet_speed;
	//等级
	int16_t level;
} Referee_t;
typedef struct
{
	int16_t x_move;
	int16_t y_move;
} shaobing_t;

void vision_RX_init();

#endif /* VISION_H_ */
