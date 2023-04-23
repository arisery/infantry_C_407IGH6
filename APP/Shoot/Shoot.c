/*
 * Shoot.c
 *
 *  Created on: Mar 5, 2023
 *      Author: arisery
 */

#include "Shoot.h"
#include "gimbal.h"
#include <CAN_Receive.h>
#include"PID.h"
#include "tim.h"
#include "string.h"
#include "key.h"
extern gimbal_t gimbal;
extern KEY_T mouse_L, mouse_R,Key_Q;
float add_angle = 50.0f;
float supply_speed_set = -200;
shoot_t shoot;
uint8_t quit_flag = 0,fric_speed_switch=0;
float R_Fric = 0.05;
float speed_low = 25, speed_normal = 40, speed_high = 60;
float fricRPM2speed = 0.005235987f;
void shoot_init()
{

	float ShootFilter[1] = { 0.02f };
	float Supply_PID_Angle[3] = { 1, 0, -0.1 }, Supply_PID_Speed[3] = { 50, 0.05, -1 };
	float Friction_PID[3] = { 600, 2.0f, -15.0 };
	shoot_speed_set(0);
	shoot.RC = get_remote_control_point();
	shoot.Supply.SupplyMotor.motor_feedback = get_shoot_Motor_Measure_Point();
	shoot.Friction.FrictionMotor[0].motor_feedback = get_friction_Motor_Measure_Point(0);
	shoot.Friction.FrictionMotor[1].motor_feedback = get_friction_Motor_Measure_Point(1);
	PID_clear(&shoot.Supply.pid_angle);
	PID_clear(&shoot.Supply.pid_speed);
	PID_Init(&shoot.Supply.pid_angle, PID_POSITION, Supply_PID_Angle, 1000, 30);
	PID_Init(&shoot.Supply.pid_speed, PID_POSITION, Supply_PID_Speed, 10000, 4000);
	shoot.Supply.SupplyMotor.offset_ecd = shoot.Supply.SupplyMotor.motor_feedback->encoder_value;
	shoot.Supply.SupplyMotor.round = 0;
	PID_Init(&shoot.Friction.pid_left, PID_POSITION, Friction_PID, 16000, 2000);
	PID_Init(&shoot.Friction.pid_right, PID_POSITION, Friction_PID, 16000, 2000);
	shoot.level = LOW;
	first_order_filter_init(&shoot.Supply.filter, 0.005, ShootFilter);
	HAL_Delay(1000);
}

/*
 * 设置发射摩擦轮转速
 * paramter 0-100
 *0：静止		100：最大转速
 */
void shoot_speed_set(uint8_t speed)
{

}

void shoot_task()
{
	shoot_mode_set(&shoot);
	shoot_data_update(&shoot);
	shoot_set_control(&shoot);
	shoot_pid_control(&shoot);

}

void shoot_mode_set(shoot_t *shoot)
{
	//更新旧模式值
	shoot->Supply.last_mode = shoot->Supply.mode;
	/*********************************************/
	//更新新模式
	if (switch_is_up(shoot->RC->rc.s[1]))
	{
		shoot->Supply.mode = kill_them;
		quit_flag = 0;
	}
	else if (switch_is_mid(shoot->RC->rc.s[1]))
	{
		shoot->Supply.mode = no_shoot;
		quit_flag = 0;
	}
	else if (switch_is_down(shoot->RC->rc.s[1]))
	{
		shoot->Supply.mode = quit;

	}
	/***********************************************/

}
void shoot_data_update(shoot_t *update)
{

	if (update == NULL)
	{
		return;
	}
	//更新旧编码器值
	update->Supply.SupplyMotor.last_encoder_value = update->Supply.SupplyMotor.encoder_value;
	//更新新编码器值
	update->Supply.SupplyMotor.encoder_value = update->Supply.SupplyMotor.motor_feedback->encoder_value;
	//更新旧角度值
	update->Supply.SupplyMotor.last_angle = update->Supply.SupplyMotor.angle;
	//更新旧角速度值
	update->Supply.SupplyMotor.last_angular_velocity = update->Supply.SupplyMotor.angular_velocity;
	//计算新的角速度值
	update->Supply.SupplyMotor.angle =
			update->Supply.SupplyMotor.round
					* 360.0f+(update->Supply.SupplyMotor.encoder_value-update->Supply.SupplyMotor.offset_ecd)*Motor_Ecd_to_Angle;
	//进行低通滤波
	first_order_filter_cali(&shoot.Supply.filter,
			(update->Supply.SupplyMotor.angle - update->Supply.SupplyMotor.last_angle) * 5.555555555f);
//	update->Supply.SupplyMotor.angular_velocity = shoot.Supply.filter.out;
	update->Supply.SupplyMotor.angular_velocity = (update->Supply.SupplyMotor.angle
			- update->Supply.SupplyMotor.last_angle) * 5.555555555f;
	//更新摩擦轮的速度
	shoot.Friction.FrictionMotor[0].speed = shoot.Friction.FrictionMotor[0].motor_feedback->speed_rpm * fricRPM2speed;

	shoot.Friction.FrictionMotor[1].speed = shoot.Friction.FrictionMotor[1].motor_feedback->speed_rpm * fricRPM2speed;

	if(Key_Q.state==key_LongPressed)
	{
		fric_speed_switch=1;

	}
	if(fric_speed_switch==1)
	{
		if(Key_Q.state==key_release)
		{
			if(speed_low==25)
			{
				speed_low=0;
			}
			else if(speed_low==0)
			{
				speed_low=25;
			}
			fric_speed_switch=0;
		}
	}
}

void shoot_set_control(shoot_t *shoot_set)
{
	if (shoot_set == NULL)
	{
		return;
	}

	shoot_speed_set(30);

	/***************************************/
	if (gimbal.mode != omnidirectional)
	{
		switch (shoot.level)
		{
			case LOW:
				shoot_set->Friction.SetSpeed = speed_low;

				break;
			case NORMAL:
				shoot_set->Friction.SetSpeed = speed_normal;

				break;
			case HIGH:
				shoot_set->Friction.SetSpeed = speed_high;

				break;
		}
	}
	else {
		shoot_set->Friction.SetSpeed = 0;
	}

	if (shoot_set->Supply.mode == kill_them)
	{
		shoot_set->Supply.SupplyMotor.angular_velocity_set = supply_speed_set;

	}
	else if (shoot_set->Supply.mode == no_shoot)
	{
		shoot_set->Supply.SupplyMotor.angular_velocity_set = 0;
	}
	/************************************/
	//左键点击
	if (shoot_set->RC->mouse.press_l == 1)
	{
		shoot_set->Supply.SupplyMotor.angular_velocity_set = supply_speed_set;
	}
	if (shoot_set->RC->mouse.press_r == 1)
	{
		shoot_set->Supply.SupplyMotor.angular_velocity_set = -10000;
	}
	if (quit_flag == 0)
	{
		shoot_set->Supply.angle_set = shoot_set->Supply.SupplyMotor.angle + 960;
	}
	//退弹标志位
	if ((shoot_set->Supply.last_mode != quit) && (shoot_set->Supply.mode == quit))
	{
		quit_flag = 1;
	}
}
void shoot_pid_control(shoot_t *shoot_pid)
{

	/*****************************************/
	//拨弹轮PID计算
	if (quit_flag == 0)
	{
		shoot_pid->Supply.set_current = PID_Calc(&shoot_pid->Supply.pid_speed,
				shoot_pid->Supply.SupplyMotor.angular_velocity, shoot_pid->Supply.SupplyMotor.angular_velocity_set);
		//在非退弹的情况下电流值是不用小于0的
//		if (shoot_pid->Supply.set_current < 0)
//		{
//			shoot_pid->Supply.set_current = 0;
//		}
	}
	else
	{
		shoot_pid->Supply.SupplyMotor.angular_velocity = (shoot_pid->Supply.SupplyMotor.angle
				- shoot_pid->Supply.SupplyMotor.last_angle) * 5.555555555f;
		shoot_pid->Supply.SupplyMotor.angular_velocity_set = PID_Calc(&shoot_pid->Supply.pid_angle,
				shoot_pid->Supply.SupplyMotor.angle, shoot_pid->Supply.angle_set);
		shoot_pid->Supply.set_current = PID_Calc(&shoot_pid->Supply.pid_speed,
				shoot_pid->Supply.SupplyMotor.angular_velocity, shoot_pid->Supply.SupplyMotor.angular_velocity_set);

	}
	/*************************************************************************/
	//摩擦轮PID计算
	shoot_pid->Friction.FrictionMotor[0].set_current = PID_Calc(&shoot_pid->Friction.pid_left,
			shoot_pid->Friction.FrictionMotor[0].speed, shoot_pid->Friction.SetSpeed);

	shoot_pid->Friction.FrictionMotor[1].set_current = PID_Calc(&shoot_pid->Friction.pid_right,
			shoot_pid->Friction.FrictionMotor[1].speed, -shoot_pid->Friction.SetSpeed);
	//发送拨弹轮电流值
	set_motor_voltage_CAN1(StdId_2006, shoot_pid->Supply.set_current, 0, 0, 0); //
}

