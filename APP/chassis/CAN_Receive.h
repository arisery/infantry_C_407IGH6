/*
 * CAN_Receive.h
 *
 *  Created on: Apr 5, 2022
 *      Author: tl
 */

#ifndef CAN_RECEIVE_H_
#define CAN_RECEIVE_H_
#include "main.h"

#define FEEDBACK_ID_BASE      0x200
#define CAN_CONTROL_ID_BASE   0x1ff
#define CAN_CONTROL_ID_EXTEND 0x2ff
#define MOTOR_MAX_NUM         4

typedef struct
{
    uint16_t encoder_value;//电机编码器反馈的机械角度值
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_encoder_value;
} motor_message_t;

void can_filter_init();
//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
 const motor_message_t *get_Chassis_Motor_Measure_Point(uint8_t i);
 const motor_message_t* get_clamp_Motor_Measure_Point(uint8_t i);
 void set_motor_voltage_CAN1( int16_t v1, int16_t v2, int16_t v3,int16_t v4);
 void set_motor_voltage_CAN2(int16_t v1, int16_t v2, int16_t v3, int16_t v4);
#endif /* CAN_RECEIVE_H_ */
