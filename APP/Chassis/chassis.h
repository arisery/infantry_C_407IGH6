/*
 * chassis.h
 *
 *  Created on: 2022年12月3日
 *      Author: arisery
 */

#ifndef CHASSIS_CHASSIS_H_
#define CHASSIS_CHASSIS_H_
#include <CAN_Receive.h>
#include "remote_control.h"
#include "PID.h"
#include"lib.h"

//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 15000.0f
#define M3505_MOTOR_SPEED_PID_KI 10.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘的移动模式
typedef enum{
	independent,//独立移动，不与其他车辆联动
	sync,//与其他车辆同步，发生相同的遥控信息
	contrary,//与其他车辆互动，发生的相反的遥控信息
}chassis_mode;


//每个轮子的相关信息
typedef struct {
	const motor_message_t *chassis_motor;
	float speed,speed_set,acceleration;
	int16_t set_current;
}chassis_wheel_t;


//整个底盘的相关信息
typedef struct {
	const RC_ctrl_t *RC;//遥控器信息
	chassis_mode move_mode,last_move_mode;
	chassis_wheel_t wheel[4];//四个轮子信息
	PID_t motor_speed_pid[4];             //底盘电机速度pid
	first_order_filter_type_t chassis_cmd_slow_set_vx;
	first_order_filter_type_t chassis_cmd_slow_set_vy;

	float vx;                         //底盘速度 前进方向 前为正，单位 m/s
	float vy;                         //底盘速度 左右方向 左为正  单位 m/s
	float wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
	float vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
	float vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
	float wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
	float chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
	float chassis_relative_angle_set; //设置相对云台控制角度
	float chassis_yaw_set;

	float vx_max_speed;  //前进方向最大速度 单位m/s
	float vx_min_speed;  //前进方向最小速度 单位m/s
	float vy_max_speed;  //左右方向最大速度 单位m/s
	float vy_min_speed;  //左右方向最小速度 单位m/s
	float wz_max_speed;	 //旋转方向最大速度 单位rad/s
	float wz_min_speed;	 //旋转方向最小速度 单位rad/s
	float chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
	float chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
	float chassis_roll;  //陀螺仪和云台电机叠加的roll角度
}chassis_struct_t;
#define RC_DEADLINE 10

//右拨档开关
#define MODE_CHANNEL1 0
//左拨档开关
#define MODE_CHANNEL2 1
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.0025f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.002f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.01f

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f
//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_WZ 2.9f
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 1
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 0
//底盘的旋转遥控通道
#define CHASSIS_WZ_CHANNEL 2


//底盘任务控制间隔 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//最大输出电流
#define MAX_MOTOR_CAN_CURRENT 16000.0f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f


//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR


//底盘摇摆按键
#define SWING_KEY    KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY    KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY    KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY   KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY    KEY_PRESSED_OFFSET_D

#define KEYBOARD_MAX_CHASSIS_SPEED_X      1.0f
#define KEYBOARD_NORMAL_CHASSIS_SPEED_X   0.5f

#define KEYBOARD_MAX_CHASSIS_SPEED_y      1.0f
#define KEYBOARD_NORMAL_CHASSIS_SPEED_y    0.5f

#define KEYBOARD_NORMAL_CHASSIS_SPEED_W		1.0f
#define KEYBOARD_MAX_CHASSIS_SPEED_W		2.5f


#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



void chassis_mode_set(chassis_struct_t* chassis_t);
void chassis_data_update(chassis_struct_t* chassis_update);
void chassis_set_contorl(chassis_struct_t *chassis_control);
void chassis_rc_to_control_vector(float *vx_set, float *vy_set,chassis_struct_t *chassis_move_rc_to_vector);
void chassis_pid_control(chassis_struct_t* chassis_pid);
void chassis_mecanum_wheel_speed(const float vx_set,const float vy_set, const float wz_set, float wheel_speed[4]);
void chassis_task();
void chassis_init(chassis_struct_t *chassis_init_t);

#endif /* CHASSIS_CHASSIS_H_ */
