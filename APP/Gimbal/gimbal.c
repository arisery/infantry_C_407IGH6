/*
 * gimbal.c
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#include "gimbal.h"
#include "main.h"
#include "Shoot.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "CAN_Receive.h"
#include "INS_task.h"
#include "function.h"
#include "stdlib.h"
float time = 0, val_an;
float temp, an, b;
gimbal_t gimbal;
float sense_x = 0.002, sense_y = 0.0015f, sense_z = 0.1;
extern IMU_t IMU_angle;
int16_t vision_rx[32], x_p;
extern CAN_HandleTypeDef hcan2;
extern shoot_t shoot;
void gimbal_init()
{
	float yaw_filter[1] =
	{ 0.06f }, pitch_filter[1] =
	{ 0.09f };
	;
	float yaw_PID_Angle[3] =
	{ 85, 0, -10 }, yaw_PID_Speed[3] =
	{ 65, 2.5, -12 }, pitch_PID_Angle[3] =
	{ 80, 0, -10 }, pitch_PID_Speed[3] =
	{ 60, 0.1, -25 };

	gimbal.test_yaw = 0;
	gimbal.RC = get_remote_control_point();
	for (int i = 0; i < 2; i++)
	{
		gimbal.axis[i].motor.motor_feedback = get_gimbal_Motor_Measure_Point(i);
		PID_clear(&gimbal.axis[i].pid_angle);
		PID_clear(&gimbal.axis[i].pid_speed);
		if (i == pitch)
		{

			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, pitch_PID_Angle,
					1000, 200);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, pitch_PID_Speed,
					25000, 6000);

		}
		else if (i == yaw)
		{
			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, yaw_PID_Angle,
					500, 10);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, yaw_PID_Speed,
					25000, 1000);
		}

	}
	gimbal.axis[pitch].motor.motor_feedback->encoder_value = 9999;
	while ((gimbal.axis[pitch].motor.motor_feedback->encoder_value > 8192)
			|| (gimbal.axis[pitch].motor.motor_feedback->encoder_value == 0))
	{

	}

	gimbal.axis[yaw].motor.offset_ecd = 7036;
	gimbal.axis[yaw].motor.angle =
			gimbal.axis[yaw].motor.round
					* 360.0f+
					(gimbal.axis[yaw].motor.motor_feedback->encoder_value-gimbal.axis[yaw].motor.offset_ecd)*Motor_Ecd_to_Angle;

	gimbal.gimbal_yaw_set = gimbal.axis[yaw].motor.angle;
	gimbal.INS_yaw_set = gimbal.INS_yaw;
	gimbal.axis[yaw].motor.round = 0;
	gimbal.axis[pitch].motor.offset_ecd = 976;
	gimbal.axis[pitch].motor.round = 0;

	first_order_filter_init(&gimbal.YAW_Filter, 0.006, yaw_filter);
	first_order_filter_init(&gimbal.PITCH_Filter, 0.006, pitch_filter);
}
/*设置云台的模式
 *
 *
 */
void gimbal_mode_set(gimbal_t *gimbal_mode)
{
	//gimbal_mode->mode=omnidirectional;

	gimbal_mode->last_mode = gimbal_mode->mode;
	if (switch_is_down(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = omnidirectional;

	}
	else if (switch_is_mid(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = Easy_Auto_Scan;
		//gimbal_mode->mode = Auto_Scan;
	}
	else if (switch_is_up(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = only_pitch;

	}

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
//		gimbal_data->axis[i].motor.angular_velocity =
//				gimbal_data->axis[i].motor.motor_feedback->speed_rpm
//						* GIMBAL_MOTOR_RPM_TO_ANGULAR_VELOCITY;
		gimbal_data->axis[i].motor.last_angle =
				gimbal_data->axis[i].motor.angle;
		gimbal_data->axis[i].motor.last_encoder_value =
				gimbal_data->axis[i].motor.encoder_value;
		gimbal_data->axis[i].motor.encoder_value =
				gimbal_data->axis[i].motor.motor_feedback->encoder_value;
		gimbal_data->axis[i].motor.angle = (gimbal_data->axis[i].motor.round
				* 360.0
				+ (gimbal_data->axis[i].motor.encoder_value
						- gimbal_data->axis[i].motor.offset_ecd)
						* Motor_Ecd_to_Angle); // yaw与pitch减速比都为1，减速比1.0f

	}
	//更新当前两个轴的弧度制角度值
	gimbal_data->gimbal_pitch = gimbal_data->axis[pitch].motor.angle;
	gimbal_data->gimbal_yaw = gimbal_data->axis[yaw].motor.angle;
	gimbal_data->INS_yaw = IMU_angle.angle;
	gimbal_data->INS_pitch = IMU_angle.pitch;

}

void gimbal_set_control(gimbal_t *gimbal_set)
{
	if (gimbal_set == NULL)
	{
		return;
	}
	//根据模式设定遥控对数值的影响
	/***********************************************************/
	if (gimbal_set->RC->keyboard.value & KEY_PRESSED_OFFSET_CTRL)
	{
		gimbal_set->mode = only_pitch;
	}
	/*********************************/
	if (gimbal_set->mode == omnidirectional)
	{
		gimbal_set->gimbal_yaw_set -= gimbal_set->RC->rc.ch[2]
				* YAW_CHANNEL_TO_ANGLE;
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3]
				* YAW_CHANNEL_TO_ANGLE;
		gimbal_set->INS_yaw_set = gimbal_set->INS_yaw;
		gimbal_set->INS_pitch_set = gimbal_set->INS_pitch;
		Pitch_Limit(gimbal_set->gimbal_pitch_set);

	}
	else if (gimbal_set->mode == only_pitch)
	{

		gimbal_set->gimbal_yaw_set = gimbal_set->gimbal_yaw;
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3]
				* YAW_CHANNEL_TO_ANGLE;
		Pitch_Limit(gimbal_set->gimbal_pitch_set);
		gimbal_set->INS_yaw_set -= gimbal_set->RC->rc.ch[2]
				* YAW_CHANNEL_TO_ANGLE;
		gimbal_set->gimbal_pitch_set += gimbal_set->RC->mouse.y * sense_y;
		if (abs(gimbal_set->RC->mouse.x) < 3)
				{
					first_order_filter_cali(&gimbal_set->YAW_Filter,
							gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE);
					gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
					gimbal_set->test_yaw -= gimbal_set->RC->rc.ch[2]
							* YAW_CHANNEL_TO_ANGLE;

				}
				else
				{
					first_order_filter_cali(&gimbal_set->YAW_Filter,
							gimbal_set->RC->mouse.x * sense_x);
					gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
					gimbal_set->INS_yaw_set -= gimbal_set->RC->mouse.x * sense_x;
					gimbal_set->test_yaw -= gimbal_set->RC->mouse.x * sense_x;

				}

	}
	else if (gimbal_set->mode == Easy_Auto_Scan)
	{

		gimbal_set->gimbal_yaw_set = gimbal_set->gimbal_yaw;
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3]
				* YAW_CHANNEL_TO_ANGLE;
		Pitch_Limit(gimbal_set->gimbal_pitch_set);
		gimbal_set->gimbal_pitch_set += gimbal_set->RC->mouse.y * sense_y;

		if (abs(gimbal_set->RC->mouse.x) < 3)
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter,
					gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->rc.ch[2]
					* YAW_CHANNEL_TO_ANGLE;

		}
		else
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter,
					gimbal_set->RC->mouse.x * sense_x);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->mouse.x * sense_x;

		}

		//
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
		shoot_speed_set(0);
	}

	//设定角度限制
//	if (gimbal_set->gimbal_yaw_set > 90)
//	{
//		gimbal_set->gimbal_yaw_set = 90.0f;
//	}
//	else if (gimbal_set->gimbal_yaw_set < -90)
//	{
//		gimbal_set->gimbal_yaw_set = -90.0f;
//	}

}

void gimbal_pid_control(gimbal_t *gimbal_pid)
{
	static uint32_t tick, last_tick, val, last_val;

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
			if (gimbal_pid->mode == Easy_Auto_Scan)
			{
				gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(
						&gimbal_pid->axis[yaw].pid_angle, gimbal_pid->INS_yaw,
						gimbal_pid->INS_yaw_set);
				gimbal_pid->axis[yaw].set_current = PID_Calc(
						&gimbal_pid->axis[i].pid_speed,
						gimbal_pid->axis[yaw].motor.angular_velocity,
						gimbal_pid->axis[yaw].motor.angular_velocity_set);
				gimbal_pid->axis[i].motor.last_angular_velocity =
						gimbal_pid->axis[i].motor.angular_velocity;
			}
			else if (gimbal_pid->mode == omnidirectional)
			{
				gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(
						&gimbal_pid->axis[yaw].pid_angle,
						gimbal_pid->gimbal_yaw, gimbal_pid->gimbal_yaw_set);
				gimbal_pid->axis[yaw].set_current = PID_Calc(
						&gimbal_pid->axis[i].pid_speed,
						gimbal_pid->axis[yaw].motor.angular_velocity,
						gimbal_pid->axis[yaw].motor.angular_velocity_set);
				gimbal_pid->axis[i].motor.last_angular_velocity =
						gimbal_pid->axis[i].motor.angular_velocity;
			}
			else if (gimbal_pid->mode == only_pitch)
			{
				gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(
						&gimbal_pid->axis[yaw].pid_angle, gimbal_pid->INS_yaw,
						gimbal_pid->INS_yaw_set);
				gimbal_pid->axis[yaw].set_current = PID_Calc(
						&gimbal_pid->axis[i].pid_speed,
						gimbal_pid->axis[yaw].motor.angular_velocity,
						gimbal_pid->axis[yaw].motor.angular_velocity_set);
				gimbal_pid->axis[i].motor.last_angular_velocity =
						gimbal_pid->axis[i].motor.angular_velocity;
			}
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

if(gimbal.mode == omnidirectional)
{
	set_motor_voltage_CAN2(StdId_6020,
				gimbal.axis[yaw].set_current,gimbal.axis[pitch].set_current, 0,0);

}
else {
	set_motor_voltage_CAN2(StdId_6020,
					gimbal.axis[yaw].set_current,gimbal.axis[pitch].set_current, shoot.Friction.FrictionMotor[0].set_current, shoot.Friction.FrictionMotor[1].set_current);

}

} //gimbal.axis[pitch].set_current

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == &huart6)
	{
		if ((vision_rx[0] == 1111) && (vision_rx[4] == 2222))
		{

			x_p = vision_rx[1];
		}
		printf("X:%d,\tY:%d\tDis:%d\r\n", vision_rx[1], vision_rx[2],
				vision_rx[3]);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &vision_rx, 32);
	}
}

void vision_RX_init()
{
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, (uint8_t*) &vision_rx, 32);

}
