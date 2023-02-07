/*
 * gimbal.c
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#include "gimbal.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "CAN_Receive.h"
float time=0,val_an;
gimbal_t gimbal;
void gimbal_init()
{	float PID_Angle[3]={12,0.05,0},PID_Speed[3]={18,3.5,0};

	gimbal.RC= get_remote_control_point();
	for(int i=0;i<2;i++)
	{
		gimbal.axis[i].motor.motor_feedback=get_gimbal_Motor_Measure_Point(i);
		PID_clear(&gimbal.axis[i].pid_angle);
		PID_clear(&gimbal.axis[i].pid_speed);
		PID_Init(&gimbal.axis[i].pid_angle,PID_POSITION,PID_Angle,700,100);
		PID_Init(&gimbal.axis[i].pid_speed,PID_POSITION,PID_Speed,30000,8000);
		gimbal.axis[i].motor.round=0;
}
	while(gimbal.axis[0].motor.offset_ecd==NULL)
		{
		gimbal.axis[0].motor.offset_ecd=gimbal.axis[0].motor.motor_feedback->encoder_value;
		}

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
		gimbal_data->axis[i].motor.last_angle=gimbal_data->axis[i].motor.angle;
		gimbal_data->axis[i].motor.last_encoder_value=gimbal_data->axis[i].motor.encoder_value;
		gimbal_data->axis[i].motor.encoder_value=gimbal_data->axis[i].motor.motor_feedback->encoder_value;
		gimbal_data->axis[i].motor.angle=gimbal_data->axis[i].motor.round*360.0f+ \
			(gimbal_data->axis[i].motor.encoder_value-gimbal_data->axis[i].motor.offset_ecd)*Motor_Ecd_to_Angle;



	}
	//更新当前两个轴的弧度制角度值
	gimbal_data->gimbal_yaw=	gimbal_data->axis[0].motor.angle;


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
	float temp,a,b;


	val =SysTick->VAL;
	tick=HAL_GetTick();
	temp=1000.0f/((float)(tick-last_tick)+((float)(val-last_val)/(float)load));
	time=temp;
	last_tick=tick;
	last_val=val;
	for(int i=0;i<2;i++)
	{

		a=(gimbal_pid->axis[i].motor.angle-gimbal_pid->axis[i].motor.last_angle)*temp;
		b=a-gimbal_pid->axis[i].motor.last_angular_velocity;
				if((b>1000)||(b<-1000))
				{
					gimbal.axis[i].motor.angular_velocity=gimbal.axis[i].motor.last_angular_velocity;
				}
				else
				{
					gimbal.axis[i].motor.angular_velocity=a;
				}
				//gimbal_pid->gimbal_yaw_set=0;
		gimbal_pid->axis[i].motor.angular_velocity_set=PID_Calc(&gimbal_pid->axis[i].pid_angle, gimbal_pid->gimbal_yaw, gimbal_pid->gimbal_yaw_set);
		//gimbal_pid->axis[i].motor.angular_velocity_set=0;
		gimbal_pid->axis[i].set_current=PID_Calc(&gimbal_pid->axis[i].pid_speed, gimbal_pid->axis[i].motor.angular_velocity, gimbal_pid->axis[i].motor.angular_velocity_set);
	gimbal_pid->axis[i].motor.last_angular_velocity=gimbal_pid->axis[i].motor.angular_velocity;

	}
	val_an=gimbal.axis[0].motor.angular_velocity;
}
float motor_ecd_to_angle_change(motor_t *motor )
{
	 if(motor==NULL)
	 {
		 return 1;
	 }
	int16_t temp;
	temp=motor->motor_feedback->encoder_value-motor->motor_feedback->last_encoder_value;

	if(temp>4000)
	{
		motor->round--;
	}
	else if(-temp>4000)
	{
		motor->round++;
	}

	return 0;
}

void gimbal_task()
{

	gimbal_mode_set(&gimbal);
	gimbal_data_update(&gimbal);
	gimbal_set_control(&gimbal);
	gimbal_pid_control(&gimbal);
	set_motor_voltage_CAN1(gimbal.axis[yaw].set_current, 0, 0, 0);

}
