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
shoot_t shoot;
/*
 *
 * 摩擦轮的PWM为TIM1的CH1 PE9和CH2 PE11
 * 50Hz， 高电平1ms为静止，2ms为最大转速
 * APB2总线，最大频率为168MHz
 * 设置PSC=1679，ARR=1999；
 *
 *
 */
void shoot_init()
{
	float PID_Angle[3] =
	{ 1, 0.2, 0 }, PID_Speed[3] =
	{ 5, 0.5, 0 };
	TIM1->PSC = 1679;
	TIM1->ARR = 1999;
	shoot_speed_set(0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);


	shoot.RC = get_remote_control_point();
	shoot.motor.motor_feedback = get_shoot_Motor_Measure_Point();
	PID_clear(&shoot.pid_angle);
	PID_clear(&shoot.pid_speed);
	PID_Init(&shoot.pid_angle, PID_POSITION, PID_Angle, 1000, 30);
	PID_Init(&shoot.pid_speed, PID_POSITION, PID_Speed, 10000, 200);
	shoot.motor.offset_ecd = shoot.motor.motor_feedback->encoder_value;
	shoot.motor.round=0;
	HAL_Delay(3000);
}

/*
 * 设置发射摩擦轮转速
 * paramter 0-100
 *0：静止		100：最大转速
 */
void shoot_speed_set(uint8_t speed)
{
	if ((speed < 0) || (speed > 100))
	{
		TIM1->CCR1 = 99;
		TIM1->CCR2 = 99;
		return;
	}
	TIM1->CCR1 = (speed + 99);
	TIM1->CCR2 = (speed + 99);
}

void shoot_task()
{
	shoot_mode_set(&shoot);
	shoot_data_update(&shoot);
	shoot_set_control(&shoot);
	shoot_pid_control(&shoot);
	set_motor_voltage_CAN1(StdId_6020, shoot.set_current, 0, 0, 0);
}

void shoot_mode_set(shoot_t *mode)
{

	mode->last_mode = mode->mode;
	if (switch_is_mid(mode->RC->rc.s[1]))
	{
		mode->mode = no_shoot;

	}

	else if (switch_is_down(mode->RC->rc.s[1]))
	{
		mode->mode = one_shoot;

	}
	else if (switch_is_up(mode->RC->rc.s[1]))
	{
		mode->mode = kill_them;

	}

}

void shoot_data_update(shoot_t *update)
{
	if (update == NULL)
	{
		return;
	}
	update->motor.last_encoder_value = update->motor.encoder_value;
	update->motor.encoder_value = update->motor.motor_feedback->encoder_value;
	update->motor.last_angle = update->motor.angle;
	update->motor.last_angular_velocity=update->motor.angular_velocity;
	update->motor.angular_velocity = update->motor.motor_feedback->speed_rpm
			* 2*PI/60.0f;
	update->motor.angle =
			update->motor.round
					* 360.0f+(update->motor.encoder_value-update->motor.offset_ecd)*Motor_Ecd_to_Angle;

}

void shoot_set_control(shoot_t *shoot_set)
{
	if (shoot_set == NULL)
	{
		return;
	}
	if (gimbal.mode == only_pitch)
	{
		if (shoot_set->mode == no_shoot)
		{

		}
		else if ((shoot_set->mode == one_shoot)
				&& (shoot_set->last_mode == no_shoot))
		{
			shoot_set->angle_set += 1440.0f;
		}
		else if (shoot_set->mode == kill_them)
		{
			shoot_set->angle_set += add_angle;
		}
	}

}
void shoot_pid_control(shoot_t * shoot_pid)
{

	shoot_pid->motor.angular_velocity_set=PID_Calc(&shoot_pid->pid_angle, shoot_pid->motor.angle, shoot_pid->angle_set);
	shoot_pid->set_current=PID_Calc(&shoot_pid->pid_speed,shoot_pid->motor.angular_velocity,shoot_pid->motor.angular_velocity_set);
}

