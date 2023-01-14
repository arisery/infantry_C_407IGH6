/*
 * gimbal.h
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#ifndef GIMBAL_GIMBAL_H_
#define GIMBAL_GIMBAL_H_

typedef enum{
pitch,//云台只有pitch轴方向运动，只能上下摆动
omnidirectional,//pitch和yaw轴都有，可以全向瞄准
omnidirectional_shoot,//全向瞄准的同时可以发射
}gimbal_mode;
typedef struct{
	gimbal_mode mode;


}gimbal_t;
#endif /* GIMBAL_GIMBAL_H_ */
