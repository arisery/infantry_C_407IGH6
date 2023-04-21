/*
 * Shoot.h
 *
 *  Created on: Mar 5, 2023
 *      Author: arisery
 */

#ifndef SHOOT_SHOOT_H_
#define SHOOT_SHOOT_H_

#include "main.h"
#include "remote_control.h"
#include "CAN_Receive.h"
#include "PID.h"
#include "Lib.h"
enum shoot_mode{
	no_shoot,//静止射击
	one_shoot,//单发
	kill_them,//连发
	quit,//退弹
};
enum BulletLevel
{
	LOW,
	NORMAL,
	HIGH,
};

typedef struct {
	enum shoot_mode mode,last_mode;
	PID_t pid_angle,pid_speed;
	motor_t SupplyMotor;
	first_order_filter_type_t filter;
	 int16_t set_current;
		float angle_set;
}Supply_t;
typedef struct {
	PID_t pid_left,pid_right;
	motor_t FrictionMotor[2];
	float  SetSpeed,speed;
	 int16_t set_current;

}Friction_t;
typedef struct{
	const RC_ctrl_t *RC;//遥控器信息
	enum BulletLevel level;
	Friction_t Friction;
	Supply_t Supply;


}shoot_t;


void shoot_init();
void shoot_task();
void shoot_mode_set(shoot_t *mode);
void shoot_data_update(shoot_t *update);
void shoot_speed_set(uint8_t speed);
void shoot_set_control(shoot_t *shoot_set);
void shoot_pid_control(shoot_t * shoot_pid);

#endif /* SHOOT_SHOOT_H_ */
