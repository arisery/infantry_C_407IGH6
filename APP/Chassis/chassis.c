/*
 * chassis.c
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#include <chassis.h>
#include <CAN_Receive.h>
#include"remote_control.h"
#include"lib.h"
#include "gimbal.h"
#include "arm_math.h"
#include "math.h"
#include "cmsis_os.h"
extern gimbal_t gimbal;
float x_W = 0.01, dis_V = 0.008, Max_follow_gimbal_w = 4.0f;
chassis_struct_t chassis;
uint32_t BIG_V, BIG_V_SET;
char EasyChassis_Flag = 0;

void Chassis_Init(chassis_struct_t *chassis_init_t)
{
	//设置底盘电机的PID参数
	const static float motor_speed_pid[3] = { M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI,
	M3505_MOTOR_SPEED_PID_KD };
	//设置底盘X，Y，W方向，跟随云台W的一阶滤波系数
	const static float chassis_x_order_filter[1] = { CHASSIS_ACCEL_X_NUM };
	const static float chassis_y_order_filter[1] = { CHASSIS_ACCEL_Y_NUM };
	const static float chassis_w_order_filter[1] = { CHASSIS_ACCEL_W_NUM };
	const static float chassis_follow_gimbal_w_filter[1] = { 0.3333f };
	//获取遥控数据地址
	chassis_init_t->RC = Get_RemoteControl_Point();
	//初始化PID

	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_init_t->wheel[i].chassis_motor = Get_ChassisMotor_MessagePoint(i);
		PID_Init(&chassis_init_t->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT,
		M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//用一阶滤波代替斜波函数生成，初始化一阶滤波
	first_order_filter_init(&chassis_init_t->chassis_cmd_slow_set_vx,
	CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_init_t->chassis_cmd_slow_set_vy,
	CHASSIS_CONTROL_TIME, chassis_y_order_filter);
	first_order_filter_init(&chassis_init_t->chassis_cmd_slow_set_vw,
	CHASSIS_CONTROL_TIME, chassis_w_order_filter);
	first_order_filter_init(&chassis_init_t->chassis_follow_gimbal_vw,
	CHASSIS_CONTROL_TIME, chassis_follow_gimbal_w_filter);
	//设置底盘最大、最小速度
	/***********************************************/
	chassis_init_t->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_init_t->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;
	/***************************************************/
	chassis_init_t->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_init_t->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	/************************************************/
	chassis_init_t->wz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_init_t->wz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	//更新一下数据
	Chassis_Update(chassis_init_t);

}

/**
 * 设置底盘的模式
 *
 *
 */
void Chassis_ModeSet(chassis_struct_t *chassis_t)
{
	//更新上次的模式
	chassis_t->last_move_mode = chassis_t->move_mode;

	/********************遥控设置模式********************************/
	if (switch_is_down(chassis_t->RC->rc.s[0]))
	{
		//跟随底盘
		chassis_t->move_mode = Follow_Chassis;

	}
	else if (switch_is_mid(chassis_t->RC->rc.s[0]))
	{
		//跟随云台
		chassis_t->move_mode = Follow_Gimbal;

	}
	else if (switch_is_up(chassis_t->RC->rc.s[0]))
	{
		//小陀螺
		chassis_t->move_mode = easy_chassis;

	}
	/********************键盘设置模式************************************/
	//按住CTRL键为小陀螺模式
	if (chassis_t->RC->keyboard.key.CTRL)
	{
		//小陀螺
		chassis_t->move_mode = easy_chassis;
	}
}

/**
 *
 * 根据四个轮子的反馈和云台角度更新底盘当前的状态数值
 *
 *
 */
void Chassis_Update(chassis_struct_t *chassis_update)
{
	if (chassis_update == NULL)
	{
		return;
	}

	for (uint8_t i = 0; i < 4; i++)
	{
		//更新电机速度，加速度是速度的PID微分
		chassis_update->wheel[i].speed =
		CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_update->wheel[i].chassis_motor->speed_rpm;
		chassis_update->wheel[i].acceleration = chassis_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
	}

	//更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	chassis_update->vx = (-chassis_update->wheel[0].speed + chassis_update->wheel[1].speed
			+ chassis_update->wheel[2].speed - chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_update->vy = (-chassis_update->wheel[0].speed - chassis_update->wheel[1].speed
			+ chassis_update->wheel[2].speed + chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_update->wz = (-chassis_update->wheel[0].speed - chassis_update->wheel[1].speed
			- chassis_update->wheel[2].speed - chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ
			/ MOTOR_DISTANCE_TO_CENTER;
}
/*
 * 设置底盘的运动速度
 *
 *
 */

void Chassis_SetControl(chassis_struct_t *chassis_control)
{
	double vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f, angle = 0, Vx = 0, Vy = 0, Vw = 0.0f;
	if (chassis_control == NULL)
	{
		return;

	}

	//****************************/
	//跟随底盘，底盘不能旋转
	if (chassis_control->move_mode == Follow_Chassis)
	{
		Chassis_RCtoControlVector(&vx_set, &vy_set, chassis_control);
		wz_set = 0.0f;
	}
	//跟随云台模式，底盘在移动时会不断旋转保持和云台同方向
	if (chassis_control->move_mode == Follow_Gimbal)
	{
		Chassis_RCtoControlVector(&Vx, &Vy, chassis_control);
		//计算当前云台与底盘的夹角，单位为rad
		angle = fmod(gimbal.gimbal_yaw, 360.0) * 2 * PI / 360.0;
		//进行角度解算，底盘会跟随云台
		vx_set = Vx * cos(angle) - Vy * (sin(angle));
		vy_set = Vx * sin(angle) + Vy * (cos(angle));
		//当前进或横移的速度太小则不进行旋转，所以可以保持底盘不动云台动，去掉之后会变成即使不移动底盘也会一直在跟随云台
		if ((fabs(Vx) < 0.05f) && (fabs(Vy) < 0.05f))
		{
			Vw = 0;
		}
		else
		{
			//计算云台与底盘的夹角，单位为 度
			angle = fmod(gimbal.gimbal_yaw, 360.0);
			//超过180实际上是负的度数，所以减去360
			if (angle > 180.0)
			{
				angle -= 360.0;
			}
			//小于-180实际上是正的度数，所以加上360
			else if (angle < -180.0)
			{
				angle += 360.0;
			}
			//让底盘旋转速度与夹角成正比
			Vw = 4.0 * angle * 2 * PI / 360.0;
			//进行旋转速度限幅，防止超功率
			if (Vw > Max_follow_gimbal_w)
			{
				Vw = Max_follow_gimbal_w;
			}
			else if (Vw < -Max_follow_gimbal_w)
			{
				Vw = -Max_follow_gimbal_w;
			}

		}
		//进行一阶滤波防止旋转速度突变超功率
		first_order_filter_cali(&chassis_control->chassis_follow_gimbal_vw, Vw);
		wz_set = chassis_control->chassis_follow_gimbal_vw.out;
		//first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vw, 0);
	}
	//小陀螺模式
	if (chassis_control->move_mode == easy_chassis)
	{
		Chassis_RCtoControlVector(&Vx, &Vy, chassis_control);
		//计算底盘与云台的夹角，单位rad
		angle = fmod(gimbal.gimbal_yaw, 360.0) * 2 * PI / 360.0;
		//速度解算
		vx_set = Vx * cos(angle) - Vy * (sin(angle));
		vy_set = Vx * sin(angle) + Vy * (cos(angle));
		first_order_filter_cali(&chassis_control->chassis_cmd_slow_set_vw, -8);
		//wz_set = chassis_control->chassis_cmd_slow_set_vw.out;
		wz_set = 8;
	}
	//输出限幅
	chassis_control->vx_set = float_constrain(vx_set, chassis_control->vx_min_speed, chassis_control->vx_max_speed);
	chassis_control->vy_set = float_constrain(vy_set, chassis_control->vy_min_speed, chassis_control->vy_max_speed);
	chassis_control->wz_set = float_constrain(wz_set, chassis_control->wz_min_speed, chassis_control->wz_max_speed);
}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void Chassis_RCtoControlVector(double *vx_set, double *vy_set, chassis_struct_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}
//遥控器原始通道值
	int16_t vx_channel, vy_channel;
	float vx_set_channel, vy_set_channel;
	//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	rc_deadline_limit(chassis_move_rc_to_vector->RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, RC_DEADLINE);
	rc_deadline_limit(chassis_move_rc_to_vector->RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, RC_DEADLINE);
//纯遥控器值
	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
//键盘操作
	if (chassis_move_rc_to_vector->RC->keyboard.key.W)
	{
		vx_set_channel = KEYBOARD_NORMAL_CHASSIS_SPEED_X;
		if (chassis_move_rc_to_vector->move_mode != easy_chassis)
		{
			if (chassis_move_rc_to_vector->RC->keyboard.key.SHIFT)
			{
//			vx_set_channel = KEYBOARD_MAX_CHASSIS_SPEED_X;
			}
		}
	}
	else if (chassis_move_rc_to_vector->RC->keyboard.key.S)
	{
		vx_set_channel = -KEYBOARD_NORMAL_CHASSIS_SPEED_X;
		if (chassis_move_rc_to_vector->move_mode != easy_chassis)
		{
			if (chassis_move_rc_to_vector->RC->keyboard.value & KEY_PRESSED_OFFSET_SHIFT)
			{
//			vx_set_channel = -KEYBOARD_MAX_CHASSIS_SPEED_X;
			}
		}
	}

	if (chassis_move_rc_to_vector->RC->keyboard.key.A)
	{
		vy_set_channel = KEYBOARD_NORMAL_CHASSIS_SPEED_y;
		if (chassis_move_rc_to_vector->move_mode != easy_chassis)
		{
			if (chassis_move_rc_to_vector->RC->keyboard.key.SHIFT)
			{
//			vy_set_channel = KEYBOARD_MAX_CHASSIS_SPEED_y;
			}
		}

	}
	else if (chassis_move_rc_to_vector->RC->keyboard.key.D)
	{
		vy_set_channel = -KEYBOARD_NORMAL_CHASSIS_SPEED_y;
		if (chassis_move_rc_to_vector->move_mode != easy_chassis)
		{
			if (chassis_move_rc_to_vector->RC->keyboard.key.SHIFT)
			{
//			vy_set_channel = -KEYBOARD_MAX_CHASSIS_SPEED_y;
			}
		}
	}

//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

	//停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

void Chassis_PID_Calculate(chassis_struct_t *chassis_pid)
{
	float wheel_speed_set[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	chassis_mecanum_wheel_speed(chassis_pid->vx_set, chassis_pid->vy_set, chassis_pid->wz_set, wheel_speed_set);
	for (uint8_t i = 0; i < 4; i++)
	{
		chassis_pid->wheel[i].speed_set = float_constrain(wheel_speed_set[i], -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
		chassis_pid->wheel[i].set_current = PID_Calc(&chassis_pid->motor_speed_pid[i], chassis_pid->wheel[i].speed,
				chassis_pid->wheel[i].speed_set);

	}

}

/*
 *
 * 由设定速度转化为实际mecanum轮的速度
 *
 */
void chassis_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
	wheel_speed[0] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[1] = vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[2] = vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

void Chassis_Task(void const *argument)
{

	osDelay(10);
	printf("Chassis Task Start...\r\n");
	osDelay(1000);

	Chassis_Init(&chassis);
	/* Infinite loop */
	for (;;)
	{

		Chassis_ModeSet(&chassis);
		Chassis_Update(&chassis);
		Chassis_SetControl(&chassis);
		Chassis_PID_Calculate(&chassis);
		if ((chassis.vx == 0) && (chassis.vy == 0) && (chassis.wz == 0) && (chassis.vx_set == 0)
				&& (chassis.vy_set == 0) && (chassis.wz_set == 0))
		{
			chassis.wheel[0].set_current = chassis.wheel[1].set_current = chassis.wheel[2].set_current =
					chassis.wheel[3].set_current = 0;
		}
		SetMotorVoltage_CAN1(StdId_3508, chassis.wheel[0].set_current, chassis.wheel[1].set_current,
				chassis.wheel[2].set_current, chassis.wheel[3].set_current);
		osDelay(5);
	}
}

