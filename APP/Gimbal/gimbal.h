/*
 * gimbal.h
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#ifndef GIMBAL_GIMBAL_H_
#define GIMBAL_GIMBAL_H_
#include "remote_control.h"
#include "CAN_Receive.h"
#include "PID.h"
#include"lib.h"
#include "Vision.h"
//#define shaobing 1
#define GIMBAL_MOTOR_RPM_TO_ANGULAR_VELOCITY 60.0f
#define YAW_CHANNEL_TO_ANGLE 0.0015151f

#ifndef PI
#define PI 3.1415926535f
#endif

#define Motor_Ecd_to_Angle 360.0f/8192.0f //      2*  PI  /8192

typedef enum{
only_pitch,//云台只有pitch轴方向运动，只能上下摆动
omnidirectional,//pitch和yaw轴都有，可以全向瞄准
omnidirectional_shoot,//全向瞄准的同时可以发射
Auto_Scan,//自瞄模式
Easy_Auto_Scan,//自瞄加小陀螺
}gimbal_mode_e;
typedef enum {
    pitch=1,
	yaw =0,

}gimbal_axis_e;
typedef struct {

PID_t	pid_angle,pid_speed;
	motor_t motor;

    float acceleration;
    int16_t set_current;

}gimbal_axis_t;

typedef struct{
	const RC_ctrl_t *RC;//遥控器信息
	gimbal_mode_e mode,last_mode;

	gimbal_axis_t axis[2];
	float gimbal_yaw,gimbal_yaw_set;
	float gimbal_pitch,gimbal_pitch_set;
	float gimbal_roll,gimbal_roll_set;
	float INS_yaw,INS_yaw_set,INS_yaw_vision_set,INS_pitch_vision_set,INS_pitch,INS_pitch_set;
	float pitch_max,pitch_min;
	float yaw_max,yaw_min;
	first_order_filter_type_t YAW_Filter,PITCH_Filter,slow_VX_set;
	float test_yaw;
	vision_t vision;
}gimbal_t;
#ifdef shaobing
#define Pitch_Limit(pitch) if ( pitch> 10.0f)  \
{											\
pitch = 10.0f;								\
}											\
else if (pitch < -30)						\
{											\
	pitch = -30.0f;							\
}
#else
#define Pitch_Limit(pitch) if ( pitch> 9.0f)		\
	{											\
	pitch = 9.0f;								\
	}											\
	else if (pitch < -20)						\
	{											\
		pitch = -20.0f;							\
	}
#endif
float motor_ecd_to_angle_change(motor_t *motor);
void Gimbal_ModeSet(gimbal_t* gimbal_mode);
void Gimbal_Update(gimbal_t* gimbal_data);
void Gimbal_SetControl(gimbal_t* gimbal_set);
void Gimbal_PID_Control(gimbal_t* gimbal_pid);
void Gimbal_Task(void const * argument);
void Gimbal_Init();

#endif /* GIMBAL_GIMBAL_H_ */
