/*
 * PID.h
 *
 *  Created on: Apr 7, 2022
 *      Author: tl
 */

#ifndef PID_H_
#define PID_H_
#include"main.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }


enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PID_t;

 void PID_Init(PID_t *pid, int mode, const float PID[3], float max_out, float max_iout);
void pid_angle_init(PID_t *pid, float kp, float ki, float kd,
		float i_max, float out_max);
void pid_speed_init(PID_t *pid, float kp, float ki, float kd,
		float i_max, float out_max);

float PID_Calc(PID_t *pid, float ref, float set);
void PID_clear(PID_t *pid);

#endif /* PID_H_ */
