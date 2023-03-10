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
extern int16_t vision_rx[32];
extern gimbal_t gimbal;
float x_W = 0.01,dis_V=0.008;
chassis_struct_t chassis;
uint32_t BIG_V, BIG_V_SET;
/**
 * 设置底盘的模式
 * 独立模式
 * 同步模式
 * 相反模式
 *
 */
void chassis_mode_set(chassis_struct_t *chassis_t)
{

	if (switch_is_down(chassis_t->RC->rc.s[0]))
	{
		chassis_t->move_mode = no_rotary;

	}
	else if (switch_is_mid(chassis_t->RC->rc.s[0]))
	{
		chassis_t->move_mode = follow_chassis;

	}
	else if (switch_is_up(chassis_t->RC->rc.s[0]))
	{
		chassis_t->move_mode = no_rotary;

	}
	chassis_t->last_move_mode = chassis_t->move_mode;

}

/**
 *
 * 根据四个轮子的反馈和云台角度更新底盘当前的状态数值
 *
 *
 */
void chassis_data_update(chassis_struct_t *chassis_update)
{
	if (chassis_update == NULL)
	{
		return;
	}

	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		//更新电机速度，加速度是速度的PID微分
		chassis_update->wheel[i].speed =
		CHASSIS_MOTOR_RPM_TO_VECTOR_SEN
				* chassis_update->wheel[i].chassis_motor->speed_rpm;
		chassis_update->wheel[i].acceleration =
				chassis_update->motor_speed_pid[i].Dbuf[0]
						* CHASSIS_CONTROL_FREQUENCE;
	}

	//更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
	chassis_update->vx = (-chassis_update->wheel[0].speed
			+ chassis_update->wheel[1].speed + chassis_update->wheel[2].speed
			- chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
	chassis_update->vy = (-chassis_update->wheel[0].speed
			- chassis_update->wheel[1].speed + chassis_update->wheel[2].speed
			+ chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
	chassis_update->wz = (-chassis_update->wheel[0].speed
			- chassis_update->wheel[1].speed - chassis_update->wheel[2].speed
			- chassis_update->wheel[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ
			/ MOTOR_DISTANCE_TO_CENTER;
	/*
	 * 底盘姿态解算，有的话添加
	 *
	 *
	 chassis_update->chassis_yaw = rad_format(*(chassis_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_update->chassis_yaw_motor->relative_angle);
	 chassis_update->chassis_pitch = rad_format(*(chassis_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_update->chassis_pitch_motor->relative_angle);
	 chassis_update->chassis_roll = *(chassis_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);
	 */
}
/*
 * 设置底盘的运动速度
 *
 *
 */

void chassis_set_contorl(chassis_struct_t *chassis_control)
{
	float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f, angle_set = 0.0f,
			wz_channel = 0.0f;
	if (chassis_control == NULL)
	{
		return;

	}
	chassis_rc_to_control_vector(&vx_set, &vy_set, chassis_control);
	rc_deadline_limit(chassis_control->RC->rc.ch[CHASSIS_WZ_CHANNEL],
			wz_channel, 5);

	if (chassis_control->move_mode == follow_chassis)
	{
		wz_set = -CHASSIS_WZ_RC_SEN * wz_channel;
		if(wz_set==0)
		{	`
		if (gimbal.mode == Auto_Scan)
		{
			wz_set = -vision_rx[1] *x_W ;
			vx_set =vision_rx[3]*dis_V;
		}
		}

	}
	else
	{
		wz_set = 0.0f;
	}

	if (chassis_control->move_mode == follow_chassis)
	{
		chassis_control->vx_set = float_constrain(vx_set,
				chassis_control->vx_min_speed, chassis_control->vx_max_speed);
		chassis_control->vy_set = float_constrain(vy_set,
				chassis_control->vy_min_speed, chassis_control->vy_max_speed);
		chassis_control->wz_set = float_constrain(wz_set,
				chassis_control->wz_min_speed, chassis_control->wz_max_speed);
	}
	else if (chassis_control->move_mode == no_rotary)
	{
		chassis_control->vx_set = float_constrain(vx_set,
				chassis_control->vx_min_speed, chassis_control->vx_max_speed);
		chassis_control->vy_set = float_constrain(vy_set,
				chassis_control->vy_min_speed, chassis_control->vy_max_speed);
		chassis_control->wz_set = float_constrain(wz_set,
				chassis_control->wz_min_speed, chassis_control->wz_max_speed);
	}
	else if (chassis_control->move_mode == contrary)
	{
		//发送反向信号
	}

}

//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(float *vx_set, float *vy_set,
		chassis_struct_t *chassis_move_rc_to_vector)
{
	if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
	{
		return;
	}
//遥控器原始通道值
	int16_t vx_channel, vy_channel;
	float vx_set_channel, vy_set_channel;
	//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
	rc_deadline_limit(chassis_move_rc_to_vector->RC->rc.ch[CHASSIS_X_CHANNEL],
			vx_channel, RC_DEADLINE);
	rc_deadline_limit(chassis_move_rc_to_vector->RC->rc.ch[CHASSIS_Y_CHANNEL],
			vy_channel, RC_DEADLINE);
//纯遥控器值
	vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;
	vy_set_channel = vy_channel * -CHASSIS_VY_RC_SEN;
//键盘操作
	if (chassis_move_rc_to_vector->RC->keyboard.value & CHASSIS_FRONT_KEY)
	{
		vx_set_channel = KEYBOARD_NORMAL_CHASSIS_SPEED_X;
		if (chassis_move_rc_to_vector->RC->keyboard.value
				& KEY_PRESSED_OFFSET_SHIFT)
			vx_set_channel = KEYBOARD_MAX_CHASSIS_SPEED_X;
	}
	else if (chassis_move_rc_to_vector->RC->keyboard.value & CHASSIS_BACK_KEY)
	{
		vx_set_channel = -KEYBOARD_NORMAL_CHASSIS_SPEED_X;
		if (chassis_move_rc_to_vector->RC->keyboard.value
				& KEY_PRESSED_OFFSET_SHIFT)
			vx_set_channel = -KEYBOARD_MAX_CHASSIS_SPEED_X;
	}

	if (chassis_move_rc_to_vector->RC->keyboard.value & CHASSIS_LEFT_KEY)
	{
		vy_set_channel = KEYBOARD_NORMAL_CHASSIS_SPEED_y;
		if (chassis_move_rc_to_vector->RC->keyboard.value
				& KEY_PRESSED_OFFSET_SHIFT)
			vy_set_channel = KEYBOARD_MAX_CHASSIS_SPEED_y;

	}
	else if (chassis_move_rc_to_vector->RC->keyboard.value & CHASSIS_RIGHT_KEY)
	{
		vy_set_channel = -KEYBOARD_NORMAL_CHASSIS_SPEED_y;
		if (chassis_move_rc_to_vector->RC->keyboard.value
				& KEY_PRESSED_OFFSET_SHIFT)
			vy_set_channel = -KEYBOARD_MAX_CHASSIS_SPEED_y;

	}
	//printf("V:%f\r\n", vx_set_channel);
	//LED4_ON;
//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx,
			vx_set_channel);
	first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy,
			vy_set_channel);

	//停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < RC_DEADLINE * CHASSIS_VX_RC_SEN
			&& vx_set_channel > -RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < RC_DEADLINE * CHASSIS_VY_RC_SEN
			&& vy_set_channel > -RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
	}

	*vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
	*vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

void chassis_pid_control(chassis_struct_t *chassis_pid)
{
	float wheel_speed_set[4] =
	{ 0.0f, 0.0f, 0.0f, 0.0f };
	uint8_t i;
	chassis_mecanum_wheel_speed(chassis_pid->vx_set, chassis_pid->vy_set,
			chassis_pid->wz_set, wheel_speed_set);

	for (i = 0; i < 4; i++)
	{
		chassis_pid->wheel[i].speed_set = float_constrain(wheel_speed_set[i],
				-MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
	}
	for (i = 0; i < 4; i++)
	{
		chassis_pid->wheel[i].set_current = PID_Calc(
				&chassis_pid->motor_speed_pid[i], chassis_pid->wheel[i].speed,
				chassis_pid->wheel[i].speed_set);
	}

}

/*
 *
 * 由设定速度转化为实际mecanum轮的速度
 *
 */
void chassis_mecanum_wheel_speed(const float vx_set, const float vy_set,
		const float wz_set, float wheel_speed[4])
{
	wheel_speed[0] = -vx_set - vy_set
			+ (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[1] = vx_set - vy_set
			+ (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
	wheel_speed[2] = vx_set + vy_set
			+ (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER
					* wz_set;
	wheel_speed[3] = -vx_set + vy_set
			+ (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER
					* wz_set;
}

void chassis_task()
{
	chassis_mode_set(&chassis);
	chassis_data_update(&chassis);
	chassis_set_contorl(&chassis);
	chassis_pid_control(&chassis);
	if ((chassis.vx == 0) && (chassis.vy == 0) && (chassis.wz == 0)
			&& (chassis.vx_set == 0) && (chassis.vy_set == 0)
			&& (chassis.wz_set == 0))
	{
		chassis.wheel[0].set_current = chassis.wheel[1].set_current =
				chassis.wheel[2].set_current = chassis.wheel[3].set_current = 0;
	}
	set_motor_voltage_CAN1(StdId_3508, chassis.wheel[0].set_current,
			chassis.wheel[1].set_current, chassis.wheel[2].set_current,
			chassis.wheel[3].set_current);

}
void chassis_init(chassis_struct_t *chassis_init_t)
{
	if (chassis_init_t == NULL)
	{
		return;
	}
	const static float motor_speed_pid[3] =
	{ M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI,
	M3505_MOTOR_SPEED_PID_KD };
	const static float chassis_x_order_filter[1] =
	{ CHASSIS_ACCEL_X_NUM };
	const static float chassis_y_order_filter[1] =
	{ CHASSIS_ACCEL_Y_NUM };
	uint8_t i;
	chassis_init_t->RC = get_remote_control_point();
	/*
	 //获取陀螺仪姿态角指针
	 chassis_init_t->chassis_INS_angle = get_INS_angle_point();
	 //获取云台电机数据指针
	 chassis_init_t->chassis_yaw_motor = get_yaw_motor_point();
	 chassis_init_t->chassis_pitch_motor = get_pitch_motor_point();
	 */
	//初始化PID
	for (i = 0; i < 4; i++)
	{
		chassis_init_t->wheel[i].chassis_motor =
				get_Chassis_Motor_Measure_Point(i);
		PID_Init(&chassis_init_t->motor_speed_pid[i], PID_POSITION,
				motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT,
				M3505_MOTOR_SPEED_PID_MAX_IOUT);
	}
	//用一阶滤波代替斜波函数生成
	first_order_filter_init(&chassis_init_t->chassis_cmd_slow_set_vx,
	CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_init_t->chassis_cmd_slow_set_vy,
	CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	//最大 最小速度
	chassis_init_t->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
	chassis_init_t->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

	chassis_init_t->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_init_t->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;

	chassis_init_t->wz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
	chassis_init_t->wz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
	//更新一下数据
	chassis_data_update(chassis_init_t);

}

