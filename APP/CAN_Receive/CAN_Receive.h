/*
 * CAN_Receive.h
 *
 *  Created on: Apr 5, 2022
 *      Author: tl
 */

#ifndef CAN_RECEIVE_H_
#define CAN_RECEIVE_H_
#include "main.h"

#define FEEDBACK_ID_BASE_3508     0x200
#define FEEDBACK_ID_BASE_2006     0x200
#define FEEDBACK_ID_BASE_6020      0x204
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         4

//电机编码值转化成角度值

#define Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192

typedef struct
{
    uint16_t encoder_value;//电机编码器反馈的机械角度值
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
	uint16_t last_encoder_value;
	void *p;

} motor_message_t;

enum motor_StdId{
	StdId_3508=0x200,
	StdId_6020=0x1FF,
	StdId_2006=0x1FF,
};
typedef struct
{	motor_message_t* motor_feedback;
	uint16_t encoder_value;
	uint16_t last_encoder_value;
	uint16_t offset_ecd;
	int16_t round;
	int16_t set_current;
    double angle ,last_angle;
    float angular_velocity;
    float last_angular_velocity;
    float angular_velocity_set;
    float speed,speed_set;
}motor_t;
void CAN_Filter_Init();
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
  motor_message_t *Get_ChassisMotor_MessagePoint(uint8_t i);
  motor_message_t* Get_GimbalMotor_MessagePoint(uint8_t i);
  motor_message_t* Get_SupplyMotor_MessagePoint();
  motor_message_t* Get_FrictionMotor_MessagePoint(uint8_t i);
  void SetMotorVoltage_CAN1(uint16_t StdId,int16_t v1, int16_t v2, int16_t v3, int16_t v4);
 void SetMotorVoltage_CAN2(uint16_t StdId,int16_t v1, int16_t v2, int16_t v3, int16_t v4);

#endif /* CAN_RECEIVE_H_ */
