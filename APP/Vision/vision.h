/*
 * vision.h
 *
 *  Created on: 2023年4月22日
 *      Author: arisery
 */

#ifndef VISION_H_
#define VISION_H_
#include "main.h"

typedef struct
{
		uint8_t header;
		uint8_t ID;
		int16_t array[4];
		uint8_t tail;
}vision_t;
void vision_RX_init();

#endif /* VISION_H_ */
