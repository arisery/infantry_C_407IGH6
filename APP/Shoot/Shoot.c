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
extern gimbal_t gimbal;
float add_angle=50.0f;
float shoot_set=400;
shoot_t shoot;
float R_Fric=0.05;
float speed_low=40,speed_normal=40,speed_high=60;
float fricRPM2speed=0.005235987f;
void shoot_init()
{
	float Supply_PID_Angle[3] =
	{ 1, 0.2, 0 }, Supply_PID_Speed[3] =
	{ 20, 0.5, 0 };
	 float Friction_PID[3]={4000,1.0f,15.0};
	shoot_speed_set(0);
	shoot.RC = get_remote_control_point();
	shoot.Supply.SupplyMotor.motor_feedback = get_shoot_Motor_Measure_Point();
	shoot.Friction.FrictionMotor[0].motor_feedback=  get_friction_Motor_Measure_Point(0);
	shoot.Friction.FrictionMotor[1].motor_feedback=  get_friction_Motor_Measure_Point(1);
	PID_clear(&shoot.Supply.pid_angle);
	PID_clear(&shoot.Supply.pid_speed);
	PID_Init(&shoot.Supply.pid_angle, PID_POSITION, Supply_PID_Angle, 1000, 30);
	PID_Init(&shoot.Supply.pid_speed, PID_POSITION, Supply_PID_Speed, 16000, 4000);
	shoot.Supply.SupplyMotor.offset_ecd = shoot.Supply.SupplyMotor.motor_feedback->encoder_value;
	shoot.Supply.SupplyMotor.round=0;
	PID_Init(&shoot.Friction.pid_left, PID_POSITION, Friction_PID, 16000, 2000);
	PID_Init(&shoot.Friction.pid_right, PID_POSITION, Friction_PID, 16000, 2000);
	shoot.level=LOW;
	HAL_Delay(3000);
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
	 set_motor_voltage_CAN1(StdId_6020,0, 0, 0, shoot.Supply.set_current);
}

void shoot_mode_set(shoot_t *shoot)
{

	shoot->Supply.last_mode = shoot->Supply.mode;
	/*********************************************/
 if (switch_is_up(shoot->RC->rc.s[1]))
	{
		shoot->Supply.mode = kill_them;

	}
 else
	{
		shoot->Supply.mode = no_shoot;

	}

 /***********************************************/


}

void shoot_data_update(shoot_t *update)
{
	if (update == NULL)
	{
		return;
	}
	update->Supply.SupplyMotor.last_encoder_value = update->Supply.SupplyMotor.encoder_value;
	update->Supply.SupplyMotor.encoder_value = update->Supply.SupplyMotor.motor_feedback->encoder_value;
	update->Supply.SupplyMotor.last_angle = update->Supply.SupplyMotor.angle;
	update->Supply.SupplyMotor.last_angular_velocity=update->Supply.SupplyMotor.angular_velocity;
	update->Supply.SupplyMotor.angular_velocity = update->Supply.SupplyMotor.motor_feedback->speed_rpm
			* 2*PI/60.0f;
	update->Supply.SupplyMotor.angle =
			update->Supply.SupplyMotor.round
					* 360.0f+(update->Supply.SupplyMotor.encoder_value-update->Supply.SupplyMotor.offset_ecd)*Motor_Ecd_to_Angle;

shoot.Friction.FrictionMotor[0].speed=shoot.Friction.FrictionMotor[0].motor_feedback->speed_rpm*fricRPM2speed;
shoot.Friction.FrictionMotor[1].speed=shoot.Friction.FrictionMotor[1].motor_feedback->speed_rpm*fricRPM2speed;

}

void shoot_set_control(shoot_t *shoot_set)
{
	if (shoot_set == NULL)
	{
		return;
	}

		shoot_speed_set(30);
		if (shoot_set->Supply.mode == no_shoot)
		{

		}
		else if ((shoot_set->Supply.mode == one_shoot)
				&& (shoot_set->Supply.last_mode == no_shoot))
		{
			shoot_set->Supply.angle_set += 1440.0f;
		}
		else if (shoot_set->Supply.mode == kill_them)
		{
			shoot_set->Supply.angle_set += add_angle;
		}


	/***************************************/
	switch (shoot.level)
	{
	case LOW :
		shoot.Friction.SetSpeed=speed_low;

		break;
	case NORMAL :
		shoot.Friction.SetSpeed=speed_normal;
		break;
	case HIGH :
		shoot.Friction.SetSpeed=speed_high;
		break;
	}

}
void shoot_pid_control(shoot_t * shoot_pid)
{

	//shoot_pid->Supply.SupplyMotor.angular_velocity_set=PID_Calc(&shoot_pid->Supply.pid_angle, shoot_pid->Supply.SupplyMotor.angle, shoot_pid->Supply.angle_set);
	if (shoot_pid->Supply.mode == kill_them)
	{
		shoot_pid->Supply.SupplyMotor.angular_velocity_set=shoot_set;

	}
	else{
		shoot_pid->Supply.SupplyMotor.angular_velocity_set=0;
	}
	/************************************/
	//左键点击
	if (shoot_pid->RC->mouse.press_l ==1 )
		{
		shoot_pid->Supply.SupplyMotor.angular_velocity_set=shoot_set;
			}
	if (shoot_pid->RC->mouse.press_r ==1 )
			{
			shoot_pid->Supply.SupplyMotor.angular_velocity_set=-100;
				}
	/*****************************************/
	shoot_pid->Supply.set_current=PID_Calc(&shoot_pid->Supply.pid_speed,shoot_pid->Supply.SupplyMotor.angular_velocity,shoot_pid->Supply.SupplyMotor.angular_velocity_set);


/*************************************************************************/
	shoot_pid->Friction.FrictionMotor[0].set_current=PID_Calc(&shoot_pid->Friction.pid_left, shoot_pid->Friction.FrictionMotor[0].speed, -shoot_pid->Friction.SetSpeed);

	shoot_pid->Friction.FrictionMotor[1].set_current=PID_Calc(&shoot_pid->Friction.pid_right, shoot_pid->Friction.FrictionMotor[1].speed, shoot_pid->Friction.SetSpeed);
	///set_motor_voltage_CAN2(StdId_3508, , 0, 0);


	//set_motor_voltage_CAN2(StdId_3508, shoot_pid->Friction.FrictionMotor[0].set_current, shoot_pid->Friction.FrictionMotor[1].set_current, 0, 0);
}

