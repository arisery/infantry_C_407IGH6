/*
 * gimbal.c
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#include "gimbal.h"
#include "main.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "CAN_Receive.h"
float time = 0, val_an;
float temp, an, b;
gimbal_t gimbal;

int16_t vision_rx[32], x_p;
void gimbal_init()
{
	float yaw_PID_Angle[3] ={ 35, 0.07, 0 }, yaw_PID_Speed[3] ={ 20, 2.5, 0 },
			pitch_PID_Angle[3] ={ 45, 0.07, 0 }, pitch_PID_Speed[3] ={ 22, 2.5, 0 };





	gimbal.RC = get_remote_control_point();
	for (int i = 0; i < 2; i++)
	{
		gimbal.axis[i].motor.motor_feedback = get_gimbal_Motor_Measure_Point(i);
		PID_clear(&gimbal.axis[i].pid_angle);
		PID_clear(&gimbal.axis[i].pid_speed);
		if (i == pitch)
		{

			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, pitch_PID_Angle,
					7200, 300);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, pitch_PID_Speed,
					30000, 15000);

		}
		else if (i == yaw)
		{
			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, yaw_PID_Angle,
					7200, 300);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, yaw_PID_Speed,
					30000, 15000);
		}
		gimbal.axis[i].motor.round = 0;
	}
	while (gimbal.axis[yaw].motor.offset_ecd == NULL)
	{
		gimbal.axis[yaw].motor.offset_ecd =
				gimbal.axis[yaw].motor.motor_feedback->encoder_value;
	}
	gimbal.axis[pitch].motor.offset_ecd = 3260;
}
/*设置云台的模式
 *
 *
 */
void gimbal_mode_set(gimbal_t *gimbal_mode)
{
	//gimbal_mode->mode=omnidirectional;

	if (switch_is_down(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = omnidirectional;

	}
	else if (switch_is_mid(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = Auto_Scan;

	}
	else if (switch_is_up(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = only_pitch;

	}
	gimbal_mode->last_mode = gimbal_mode->mode;

}
/*
 * 根据云台电机反馈数据更新当前云台数据
 *
 */
void gimbal_data_update(gimbal_t *gimbal_data)
{
	if (gimbal_data == NULL)
	{
		return;
	}

	//更新两个轴的角速度，将转速转换为弧度制的角速度
	for (int i = 0; i < 2; i++)
	{
		gimbal_data->axis[i].motor.angular_velocity =
				gimbal_data->axis[i].motor.motor_feedback->speed_rpm
						* GIMBAL_MOTOR_RPM_TO_ANGULAR_VELOCITY;
		gimbal_data->axis[i].motor.last_angle =
				gimbal_data->axis[i].motor.angle;
		gimbal_data->axis[i].motor.last_encoder_value =
				gimbal_data->axis[i].motor.encoder_value;
		gimbal_data->axis[i].motor.encoder_value =
				gimbal_data->axis[i].motor.motor_feedback->encoder_value;
		gimbal_data->axis[i].motor.angle =
				gimbal_data->axis[i].motor.round
						* 360.0f+
						(gimbal_data->axis[i].motor.encoder_value-gimbal_data->axis[i].motor.offset_ecd)*Motor_Ecd_to_Angle;

	}
	//更新当前两个轴的弧度制角度值
	gimbal_data->gimbal_pitch = gimbal_data->axis[pitch].motor.angle;
	gimbal_data->gimbal_yaw = gimbal_data->axis[yaw].motor.angle;

}

void gimbal_set_control(gimbal_t *gimbal_set)
{
	if (gimbal_set == NULL)
	{
		return;
	}
	if (gimbal_set->mode == omnidirectional)
	{
		gimbal_set->gimbal_yaw_set -= gimbal_set->RC->rc.ch[2]
				* YAW_CHANNEL_TO_ANGLE;
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3]
				* YAW_CHANNEL_TO_ANGLE;

	}
	else if (gimbal_set->mode == only_pitch)
	{
		gimbal_set->gimbal_pitch_set += gimbal_set->RC->rc.ch[3]
				* YAW_CHANNEL_TO_ANGLE;
	}
	else if (gimbal_set->mode == Auto_Scan)
	{
		if (vision_rx[1 != 0])
		{
			gimbal_set->gimbal_yaw_set = gimbal_set->gimbal_yaw
					- ((float) vision_rx[1] / 50.0f);
			gimbal_set->gimbal_pitch_set = gimbal_set->gimbal_pitch
					- ((float) vision_rx[2] / 50.0f);
		}
	}
	if (gimbal_set->gimbal_yaw_set > 90)
	{
		gimbal_set->gimbal_yaw_set = 90.0f;
	}
	else if (gimbal_set->gimbal_yaw_set < -90)
	{
		gimbal_set->gimbal_yaw_set = -90.0f;
	}
	if (gimbal_set->gimbal_pitch_set > 13)
	{
		gimbal_set->gimbal_pitch_set = 13.0f;
	}
	else if (gimbal_set->gimbal_pitch_set < -50)
	{
		gimbal_set->gimbal_pitch_set = -50.0f;
	}
}

void gimbal_pid_control(gimbal_t *gimbal_pid)
{
	static uint32_t tick, last_tick, val, last_val;
	static uint32_t load = 168000;

	val = SysTick->VAL;
	tick = HAL_GetTick();
	temp = 1000.0f / 5.0f; //((float)(tick-last_tick)+((float)(val-last_val)/(float)load));
	time = temp;
	last_tick = tick;
	last_val = val;
	for (int i = 0; i < 2; i++)
	{

		an = (gimbal_pid->axis[i].motor.angle
				- gimbal_pid->axis[i].motor.last_angle) * 200.0f;
		b = an - gimbal_pid->axis[i].motor.last_angular_velocity;
		if ((b > 8000) || (b < -8000))
		{
			gimbal.axis[i].motor.angular_velocity =
					gimbal.axis[i].motor.last_angular_velocity;
		}
		else
		{
			gimbal.axis[i].motor.angular_velocity = an;
		}
		//gimbal_pid->gimbal_yaw_set=0;
		if (i == yaw)
		{
			gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(
					&gimbal_pid->axis[yaw].pid_angle, gimbal_pid->gimbal_yaw,
					gimbal_pid->gimbal_yaw_set);
			gimbal_pid->axis[yaw].set_current = PID_Calc(
					&gimbal_pid->axis[i].pid_speed,
					gimbal_pid->axis[yaw].motor.angular_velocity,
					gimbal_pid->axis[yaw].motor.angular_velocity_set);
			gimbal_pid->axis[i].motor.last_angular_velocity =
					gimbal_pid->axis[i].motor.angular_velocity;
		}
		else if (i == pitch)
		{
			gimbal_pid->axis[pitch].motor.angular_velocity_set = PID_Calc(
					&gimbal_pid->axis[pitch].pid_angle,
					gimbal_pid->gimbal_pitch, gimbal_pid->gimbal_pitch_set);
			gimbal_pid->axis[pitch].set_current = PID_Calc(
					&gimbal_pid->axis[pitch].pid_speed,
					gimbal_pid->axis[pitch].motor.angular_velocity,
					gimbal_pid->axis[pitch].motor.angular_velocity_set);
			gimbal_pid->axis[pitch].motor.last_angular_velocity =
					gimbal_pid->axis[pitch].motor.angular_velocity;
		}
		//gimbal_pid->axis[i].motor.angular_velocity_set=0;

	}
	val_an = gimbal.axis[0].motor.encoder_value
			- gimbal.axis[0].motor.last_encoder_value;
}
float motor_ecd_to_angle_change(motor_t *motor)
{
	if (motor == NULL)
	{
		return 1;
	}
	int16_t temp;
	temp = motor->motor_feedback->encoder_value
			- motor->motor_feedback->last_encoder_value;

	if (temp > 4000)
	{
		motor->round--;
	}
	else if (-temp > 4000)
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
	set_motor_voltage_CAN2(gimbal.axis[pitch].set_current,
			gimbal.axis[yaw].set_current, 0, 0);

}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart = &huart6)
	{
		if ((Size == 8) && (vision_rx[0] == 1111) && (vision_rx[3] == 2222))
		{
			printf("X:%d,\tY:%d\r\n", vision_rx[1], vision_rx[2]);
			x_p = vision_rx[1];
		}

		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, vision_rx, 32);
	}
}

void vision_RX_init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, vision_rx, 32);

}
