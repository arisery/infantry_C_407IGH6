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
#include "vision.h"
#include "string.h"
#include "key.h"
#include "Vision.h"
#include "math.h"
#include "cmsis_os.h"
float time = 0, val_an;
float temp, an, b, vision_sense_x = 50.0, vision_sense_y = 200.0;
uint8_t vision_counter = 0;
gimbal_t gimbal;
uint8_t rotary = 0;
float sense_x = 0.002, sense_y = 0.0015f, sense_z = 0.1;
uint8_t Auto_flag = 0;
extern IMU_t IMU_angle;
extern CAN_HandleTypeDef hcan2;
extern shoot_t shoot;
extern KEY_T btn[1], mouse_L, mouse_R, Key_shift;
extern MSG_t MSG;
float swim = 0;
void Gimbal_Init()
{
	//云台一阶滤波系数设置
	float yaw_filter[1] = { 0.06f }, pitch_filter[1] = { 0.09f };
	//云台PID参数设置，yaw和pitch轴各有角度环和速度环
	float yaw_PID_Angle[3] = { 45, 0, -20 }, yaw_PID_Speed[3] = { 115, 2.5, -40 }, pitch_PID_Angle[3] = { 80, 0, -10 },
			pitch_PID_Speed[3] = { 60, 0.1, -25 };
	//一阶滤波初始化
	first_order_filter_init(&gimbal.YAW_Filter, 0.006, yaw_filter);
	first_order_filter_init(&gimbal.PITCH_Filter, 0.006, pitch_filter);
	gimbal.test_yaw = 0;
	//获取遥控数据地址
	gimbal.RC = Get_RemoteControl_Point();

	for (int i = 0; i < 2; i++)
	{
		//获取电机信息数据地址
		gimbal.axis[i].motor.motor_feedback = Get_GimbalMotor_MessagePoint(i);
		//清除PID数据
		PID_clear(&gimbal.axis[i].pid_angle);
		PID_clear(&gimbal.axis[i].pid_speed);
		if (i == pitch)
		{
			//PID双环初始化
			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, pitch_PID_Angle, 1000, 200);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, pitch_PID_Speed, 25000, 6000);

		}
		else if (i == yaw)
		{
			PID_Init(&gimbal.axis[i].pid_angle, PID_POSITION, yaw_PID_Angle, 500, 0);
			PID_Init(&gimbal.axis[i].pid_speed, PID_POSITION, yaw_PID_Speed, 25000, 1000);
		}

	}
	//先给一个大的值，当电机更新数据时才能恢复，防止电机未上电导致数据有问题
	gimbal.axis[pitch].motor.motor_feedback->encoder_value = 9999;
	while ((gimbal.axis[pitch].motor.motor_feedback->encoder_value > 8192)
			|| (gimbal.axis[pitch].motor.motor_feedback->encoder_value == 0))
	{

	}
	//将pitch轴和yaw轴的圈数都归零，否则上电可能检测到圈数导致疯云台
	gimbal.axis[yaw].motor.round = 0;
	gimbal.axis[pitch].motor.round = 0;

	//设定初始云台编码器值，当电机硬件结构改变时都需要重新测量，且必须关闭电机发送电流数据，小心机械伤人
#ifdef shaobing
	gimbal.axis[yaw].motor.offset_ecd = 7115;
	gimbal.axis[pitch].motor.offset_ecd = 3400;
#else
	gimbal.axis[yaw].motor.offset_ecd = 7045; //  7120
	gimbal.axis[pitch].motor.offset_ecd = 6612; //3260
#endif
	//更新一下YAW轴角度
	gimbal.axis[yaw].motor.angle = gimbal.axis[yaw].motor.round * 360.0f+
	(gimbal.axis[yaw].motor.motor_feedback->encoder_value-gimbal.axis[yaw].motor.offset_ecd)*Motor_Ecd_to_Angle;
	//更新YAW轴目标值，防止上电yaw转到0处，小心机械伤人
	gimbal.gimbal_yaw_set = gimbal.axis[yaw].motor.angle;
	gimbal.INS_yaw_set = gimbal.INS_yaw;
	//初始化云台模式
	gimbal.mode = omnidirectional;
}
/*
 *
 *设置云台的模式
 */
void Gimbal_ModeSet(gimbal_t *gimbal_mode)
{

	//更新旧模式
	gimbal_mode->last_mode = gimbal_mode->mode;
	//根据遥控状态确定模式
	//这三个模式的名字与实际功能并无实际联系
	if (switch_is_down(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = omnidirectional;

	}
	else if (switch_is_mid(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = Easy_Auto_Scan;

	}
	else if (switch_is_up(gimbal_mode->RC->rc.s[0]))
	{
		gimbal_mode->mode = only_pitch;

	}
	else
	{
		gimbal_mode->mode = Easy_Auto_Scan;
	}
	/*****************************************************************/
	//根据键盘按键设置模式
	if (gimbal_mode->RC->keyboard.key.CTRL)
	{
		gimbal_mode->mode = only_pitch;
	}
}
/*
 * 根据云台电机反馈数据更新当前云台数据
 *
 */
void Gimbal_Update(gimbal_t *gimbal_data)
{
	if (gimbal_data == NULL)
	{
		return;
	}

	//更新两个轴的角速度，将转速转换为弧度制的角速度
	for (int i = 0; i < 2; i++)
	{
//		原本使用电机反馈的RPM作为角速度
//		gimbal_data->axis[i].motor.angular_velocity =
//				gimbal_data->axis[i].motor.motor_feedback->speed_rpm
//						* GIMBAL_MOTOR_RPM_TO_ANGULAR_VELOCITY;
		//更新旧编码器值
		gimbal_data->axis[i].motor.last_encoder_value = gimbal_data->axis[i].motor.encoder_value;
		//更新编码器值
		gimbal_data->axis[i].motor.encoder_value = gimbal_data->axis[i].motor.motor_feedback->encoder_value;
		//更新旧角度值
		gimbal_data->axis[i].motor.last_angle = gimbal_data->axis[i].motor.angle;
		//更新角度值
		gimbal_data->axis[i].motor.angle = (gimbal_data->axis[i].motor.round * 360.0
				+ (gimbal_data->axis[i].motor.encoder_value - gimbal_data->axis[i].motor.offset_ecd)
						* Motor_Ecd_to_Angle); // yaw与pitch减速比都为1，减速比1.0f

	}
	//更新当前两个轴的角度值
	gimbal_data->gimbal_pitch = gimbal_data->axis[pitch].motor.angle;
	gimbal_data->gimbal_yaw = gimbal_data->axis[yaw].motor.angle;
	//更新陀螺仪的角度值
	gimbal_data->INS_yaw = IMU_angle.angle;
	gimbal_data->INS_pitch = IMU_angle.pitch;

}

void Gimbal_SetControl(gimbal_t *gimbal_set)
{
	if (gimbal_set == NULL)
	{
		return;
	}
	//根据模式设定遥控对数值的影响
	/***********************************************************/

	//云台全向，但是不依靠陀螺仪，只靠本身云台编码器获取角度
	if (gimbal_set->mode == omnidirectional)
	{
		//将陀螺仪目标设置与陀螺仪角度保持一致
		gimbal_set->INS_yaw_set = gimbal_set->INS_yaw;
		gimbal_set->INS_pitch_set = gimbal_set->INS_pitch;

		//遥控值变化
		gimbal_set->gimbal_yaw_set -= gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE;
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3] * YAW_CHANNEL_TO_ANGLE;
		//键鼠控制
		//yaw轴
		first_order_filter_cali(&gimbal_set->YAW_Filter, gimbal_set->RC->mouse.x * sense_x);
		gimbal_set->gimbal_yaw_set -= gimbal_set->YAW_Filter.out;
		gimbal_set->test_yaw -= gimbal_set->RC->mouse.x * sense_x;
		//pitch轴
		gimbal_set->gimbal_pitch_set += gimbal_set->RC->mouse.y * sense_y;
		//角度限位
		Pitch_Limit(gimbal_set->gimbal_pitch_set);

	}
	else if (gimbal_set->mode == Easy_Auto_Scan)
	{
		//将机械角度目标设置与机械角度保持一致
		gimbal_set->gimbal_yaw_set = gimbal_set->gimbal_yaw;
		//yaw轴
		if (abs(gimbal_set->RC->mouse.x) < 3)
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter, gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE;

		}
		else
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter, gimbal_set->RC->mouse.x * sense_x);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->mouse.x * sense_x;

		}
		//pitch轴
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3] * YAW_CHANNEL_TO_ANGLE;

		gimbal_set->gimbal_pitch_set += gimbal_set->RC->mouse.y * sense_y;
		//	角度限位
		Pitch_Limit(gimbal_set->gimbal_pitch_set);

	}
	else if (gimbal_set->mode == only_pitch)
	{

		//将机械角度目标设置与机械角度保持一致
		gimbal_set->gimbal_yaw_set = gimbal_set->gimbal_yaw;
		//yaw轴
		if (abs(gimbal_set->RC->mouse.x) < 3)
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter, gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->rc.ch[2] * YAW_CHANNEL_TO_ANGLE;

		}
		else
		{
			first_order_filter_cali(&gimbal_set->YAW_Filter, gimbal_set->RC->mouse.x * sense_x);
			gimbal_set->INS_yaw_set -= gimbal_set->YAW_Filter.out;
			gimbal_set->test_yaw -= gimbal_set->RC->mouse.x * sense_x;

		}
		//pitch轴
		gimbal_set->gimbal_pitch_set -= gimbal_set->RC->rc.ch[3] * YAW_CHANNEL_TO_ANGLE;

		gimbal_set->gimbal_pitch_set += gimbal_set->RC->mouse.y * sense_y;
		//	角度限位
		Pitch_Limit(gimbal_set->gimbal_pitch_set);

	}

	Pitch_Limit(gimbal_set->gimbal_pitch_set);
	//设定角度限制


}
void Gimbal_PID_Control(gimbal_t *gimbal_pid)
{

for (int i = 0; i < 2; i++)
{

	an = (gimbal_pid->axis[i].motor.angle - gimbal_pid->axis[i].motor.last_angle) * 200.0f;
	b = an - gimbal_pid->axis[i].motor.last_angular_velocity;
	if ((b > 8000) || (b < -8000))
	{
		gimbal.axis[i].motor.angular_velocity = gimbal.axis[i].motor.last_angular_velocity;
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

			gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(&gimbal_pid->axis[yaw].pid_angle,
					gimbal_pid->INS_yaw, gimbal_pid->INS_yaw_set);

			gimbal_pid->axis[yaw].set_current = PID_Calc(&gimbal_pid->axis[i].pid_speed,
					gimbal_pid->axis[yaw].motor.angular_velocity, gimbal_pid->axis[yaw].motor.angular_velocity_set);
			gimbal_pid->axis[i].motor.last_angular_velocity = gimbal_pid->axis[i].motor.angular_velocity;
		}

		else if (gimbal_pid->mode == omnidirectional)
		{
			gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(&gimbal_pid->axis[yaw].pid_angle,
					gimbal_pid->gimbal_yaw, gimbal_pid->gimbal_yaw_set);
			gimbal_pid->axis[yaw].set_current = PID_Calc(&gimbal_pid->axis[i].pid_speed,
					gimbal_pid->axis[yaw].motor.angular_velocity, gimbal_pid->axis[yaw].motor.angular_velocity_set);
			gimbal_pid->axis[i].motor.last_angular_velocity = gimbal_pid->axis[i].motor.angular_velocity;
		}
		else if (gimbal_pid->mode == only_pitch)
		{

			gimbal_pid->axis[yaw].motor.angular_velocity_set = PID_Calc(&gimbal_pid->axis[yaw].pid_angle,
					gimbal_pid->INS_yaw, gimbal_pid->INS_yaw_set);
			gimbal_pid->axis[yaw].set_current = PID_Calc(&gimbal_pid->axis[i].pid_speed,
					gimbal_pid->axis[yaw].motor.angular_velocity, gimbal_pid->axis[yaw].motor.angular_velocity_set);
			gimbal_pid->axis[i].motor.last_angular_velocity = gimbal_pid->axis[i].motor.angular_velocity;

		}
	}
	else if (i == pitch)
	{

		gimbal_pid->axis[pitch].motor.angular_velocity_set = PID_Calc(&gimbal_pid->axis[pitch].pid_angle,
				gimbal_pid->gimbal_pitch, gimbal_pid->gimbal_pitch_set);
		gimbal_pid->axis[pitch].set_current = PID_Calc(&gimbal_pid->axis[pitch].pid_speed,
				gimbal_pid->axis[pitch].motor.angular_velocity, gimbal_pid->axis[pitch].motor.angular_velocity_set);
		gimbal_pid->axis[pitch].motor.last_angular_velocity = gimbal_pid->axis[pitch].motor.angular_velocity;

	}

}
}
/*
 * 电机圈数判定
 * 假设电机在两次判定中转过的角度不会超过半圈，那么只要编码器值的变化超过半圈也就是大概4000则可以确定电机经过了零点，圈数变化
 */
float motor_ecd_to_angle_change(motor_t *motor)
{
if (motor == NULL)
{
	return 1;
}
int16_t temp;
temp = motor->motor_feedback->encoder_value - motor->motor_feedback->last_encoder_value;

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

void Gimbal_Task(void const *argument)
{
printf("Gimbal Task Start...\r\n");
osDelay(1000);
Gimbal_Init();

/* Infinite loop */
for (;;)
{

	Gimbal_ModeSet(&gimbal);
	Gimbal_Update(&gimbal);
	Gimbal_SetControl(&gimbal);
	Gimbal_PID_Control(&gimbal);
#ifdef shaobing
	if (gimbal.mode == omnidirectional)
	{
		SetMotorVoltage_CAN2(StdId_6020, gimbal.axis[yaw].set_current, gimbal.axis[pitch].set_current, 0, 0);
//
	}
	else
	{
		SetMotorVoltage_CAN2(StdId_6020, gimbal.axis[yaw].set_current, gimbal.axis[pitch].set_current,
				shoot.Friction.FrictionMotor[0].set_current, shoot.Friction.FrictionMotor[1].set_current);

	}


#else
	if (gimbal.mode == omnidirectional)
	{
		SetMotorVoltage_CAN2(StdId_6020, gimbal.axis[yaw].set_current, -gimbal.axis[pitch].set_current, 0, 0);
//
	}
	else
	{
		SetMotorVoltage_CAN2(StdId_6020, gimbal.axis[yaw].set_current, -gimbal.axis[pitch].set_current,
				shoot.Friction.FrictionMotor[0].set_current, shoot.Friction.FrictionMotor[1].set_current);
		//	set_motor_voltage_CAN2(StdId_6020, gimbal.axis[yaw].set_current, gimbal.axis[pitch].set_current, 0, 0);

	}
#endif

	osDelay(5);
}
}
