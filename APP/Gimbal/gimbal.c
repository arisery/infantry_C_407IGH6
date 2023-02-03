/*
 * gimbal.c
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#include "gimbal.h"
#include "main.h"
#include "stm32f4xx_hal.h"
gimbal_t gimbal;
void gimbal_init()
{

}
/*设置云台的模式
 *
 *
 */
void gimbal_mode_set(gimbal_t* gimbal_mode)
{
	gimbal_mode->mode=omnidirectional;
	/*
	if (switch_is_down(gimbal->RC->rc.s[0]))
		{
			gimbal->move_mode = independent;

		}
		else if (switch_is_mid(gimbal->RC->rc.s[0]))
		{
			gimbal->move_mode = sync;

		}
		else if (switch_is_up(gimbal->RC->rc.s[0]))
		{
			gimbal->move_mode = contrary;

		}
		gimbal->last_move_mode = gimbal->move_mode;

	*/
}
/*
 * 根据云台电机反馈数据更新当前云台数据
 *
 */
void gimbal_data_update(gimbal_t* gimbal_data)
{
	if(gimbal_data==NULL)
	{
		return ;
	}

	//更新两个轴的角速度，将转速转换为弧度制的角速度
	for(int i=0;i<2;i++)
	{
		gimbal_data->axis[i].motor.angular_velocity=gimbal_data->axis[i].motor.motor_feedback->speed_rpm*GIMBAL_MOTOR_RPM_TO_ANGULAR_VELOCITY;
	}
	//更新当前两个轴的弧度制角度值
	gimbal_data->gimbal_yaw=gimbal_data->axis[yaw].motor.encoder_value;


}

void gimbal_set_control(gimbal_t* gimbal_set)
{
	if(gimbal_set==NULL)
	{
		return ;
	}
	if(gimbal_set->mode==omnidirectional)
	{
		gimbal_set->gimbal_yaw_set+=gimbal_set->RC->rc.ch[2]*YAW_CHANNEL_TO_ANGLE;
		gimbal_set->gimbal_pitch_set+=gimbal_set->RC->rc.ch[3]*YAW_CHANNEL_TO_ANGLE;
	}
	else if(gimbal_set->mode==only_pitch)
	{
		gimbal_set->gimbal_pitch_set+=gimbal_set->RC->rc.ch[3]*YAW_CHANNEL_TO_ANGLE;
	}
}

void gimbal_pid_control(gimbal_t* gimbal_pid)
{
	static uint32_t tick,last_tick,val,last_val;
	static uint32_t load=168000;
	float temp;


	val =SysTick->VAL;
	tick=HAL_GetTick();
	temp=1000/((tick-last_tick)+((val-last_val)/load));
	last_tick=tick;
	last_val=val;
	for(int i=0;i<2;i++)
	{
		gimbal_pid->axis[i].motor.angular_velocity=(gimbal_pid->axis[i].motor.angle-gimbal_pid->axis[i].motor.last_angle)*temp;
		gimbal_pid->axis[i].motor.angular_velocity_set=PID_Calc(&gimbal_pid->axis[i].pid_angle, gimbal_pid->gimbal_yaw, gimbal_pid->gimbal_yaw_set);
		gimbal_pid->axis[i].set_current=PID_Calc(&gimbal_pid->axis[i].pid_speed, gimbal_pid->axis[i].motor.angular_velocity, gimbal_pid->axis[i].motor.angular_velocity_set);
	}
}
float motor_ecd_to_angle_change(motor_t *motor )
{
	 if(motor==NULL)
	 {
		 return 1;
	 }
	int16_t temp;
	temp=motor->encoder_value-motor->last_encoder_value;

	if(temp>4000)
	{
		motor->round--;
	}
	else if(-temp>4000)
	{
		motor->round++;
	}
	motor->angle=motor->round*2*PI+(motor->encoder_value-motor->offset_ecd)*Motor_Ecd_to_Rad;

	return 0;
}

void gimbal_task()
{

	gimbal_mode_set(&gimbal);
	gimbal_data_update(&gimbal);
	gimbal_set_control(&gimbal);
	gimbal_pid_control(&gimbal);
	set_motor_voltage_CAN2(gimbal.axis[yaw].set_current, gimbal.axis[pitch].set_current, 0, 0);

}
