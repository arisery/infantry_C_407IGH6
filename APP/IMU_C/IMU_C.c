/*
 * IMU_C.h
 *
 *  Created on: 2023年1月30日
 *      Author: Phoenix
 */
#include "IMU_C.h"
#include "tim.h"
#define IST8310_IN_WORK
#undef IST8310_IN_WORK
static IMU_Typedef g_imu_struct = {

		.ax_raw = 0,
		.ay_raw = 0,
		.az_raw = 0,
		.gx_raw = 0,
		.gy_raw = 0,
		.gz_raw = 0,
		.mx_raw = 0,
		.my_raw = 0,
		.mz_raw = 0,
		.gx_offset = 0,
		.gy_offset = 0,
		.gz_offset = 0,
		.quat.q0 = 1,
		.quat.q1 = 0,
		.quat.q2 = 0,
		.quat.q3 = 0,
		.Kp = 2.0f,
		.Ki = 0.1f
};
static void Get_Gyro_StaticError(void);
static void IMU_GetInit_Angle(void);
static void Init_Quaternions(void);
static void IMU_GetData(void);
static void IMU_Temperature_Compensate(void);

static Status_Typedef IMU_START_STATUS = Status_ERROR;
/**
  * @brief :IMU初始化函数
  * @param :无
  * @note  :无
  * @retval:IMU_ERROR_Typedef类型数据，可用于定位发生错误位置
  */
IMU_ERROR_Typedef IMU_Init(void)
{
	IMU_ERROR_Typedef imu_error = IMU_NO_ERROR;
	imu_error |= BMI088_Init();
	imu_error |= IST8310_Init();

	return imu_error;
}

void IMU_Start(void)
{

				Get_Gyro_StaticError();
				Init_Quaternions();
				IMU_START_STATUS = Status_OK;

				/*
	while (IMU_START_STATUS == Status_ERROR)
	{
		if (fabs(g_imu_struct.temperature - IMU_MAX_TEMP_SET) <= 0.2)
		{

		}
	}*/
}

static void Get_Gyro_StaticError(void)
{
	const uint16_t SAMPLES_COUNT = 5000;
	int16_t gyro_error_data[3];
	uint16_t i;
	for (i = 0; i < SAMPLES_COUNT; i++)
	{
		BMI088_Getdata_Gyro_raw(gyro_error_data);
		g_imu_struct.gx_offset += gyro_error_data[0];
		g_imu_struct.gy_offset += gyro_error_data[1];
		g_imu_struct.gz_offset += gyro_error_data[2];
		HAL_Delay(1);
	}
	 g_imu_struct.gx_offset /=  SAMPLES_COUNT;
	 g_imu_struct.gy_offset /=  SAMPLES_COUNT;
	 g_imu_struct.gz_offset /=  SAMPLES_COUNT;
}

static void IMU_GetInit_Angle(void)
{

	float temp = 0;

	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	IMU_GetData();

	float ax = g_imu_struct.ax_raw;
	float ay = g_imu_struct.ay_raw;
	float az = g_imu_struct.az_raw;
	float mx = g_imu_struct.mx_raw;
	float my = g_imu_struct.my_raw;
	float mz = g_imu_struct.mz_raw;

	temp = 1 / invSqrt( pow(ay, 2) + pow(az, 2));
	roll = atan2f(ay, az);
	pitch = -atan2f(ax, temp);
#ifdef IST8310_IN_WORK
	mx = mx * cos(roll) + my * sin(roll) * sin(pitch) + mz * sin(roll) * cos(pitch);
	my = my * cos(pitch) - mz * sin(pitch);
	yaw = -atan2f(my, mx);//此处为负结果才是正确的
#endif
	g_imu_struct.roll = roll;
    g_imu_struct.pitch = pitch;
    g_imu_struct.yaw = yaw;

}

static void Init_Quaternions(void)
{
	float roll = 0;
	float pitch = 0;
	float yaw = 0;

	IMU_GetInit_Angle();

	roll = g_imu_struct.roll;
	pitch = g_imu_struct.pitch;
#ifdef IST8310_IN_WORK
	yaw = g_imu_struct.yaw;
#endif


	g_imu_struct.quat.q0 = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
	g_imu_struct.quat.q1 = sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2);
	g_imu_struct.quat.q2 = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
	g_imu_struct.quat.q3 = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*cos(yaw / 2);


}
static void IMU_GetData(void)
{
	int16_t accdata[3];
	int16_t gyrodata[3];
	int16_t magdata[3];
	BMI088_Getdata_Acc_raw(accdata);
	BMI088_Getdata_Gyro_raw(gyrodata);
	IST8310_Getdata_Mag_raw(magdata);
	g_imu_struct.temperature =  BMI088_Get_Temperature();
	g_imu_struct.ax_raw = accdata[0];
	g_imu_struct.ay_raw = accdata[1];
	g_imu_struct.az_raw = accdata[2];
	g_imu_struct.gx_raw = gyrodata[0];
	g_imu_struct.gy_raw = gyrodata[1];
	g_imu_struct.gz_raw = gyrodata[2];
	g_imu_struct.mx_raw = magdata[0];
	g_imu_struct.my_raw = magdata[1];
	g_imu_struct.mz_raw = magdata[2];
}

//三传感器显式互补滤波数据融合解算姿态
void IMU_Data_Fusion_Mahony(float dt, float *roll, float *pitch, float *yaw)
{

	IMU_GetData();
	IMU_Temperature_Compensate();
	//未开始执行IMU功能,直接返回
	if (IMU_START_STATUS == Status_ERROR)
	{
		return;
	}


		//互补滤波系数
//	static float Kp = 2.0;
//	static float Ki = 0.01;

	float norm_temp;
	//
	float vx, vy, vz;
	float hx, hy, hz, bx, bz, wx, wy, wz;

	float ex, ey, ez;
	static float ex_sum = 0;
	static float ey_sum = 0;
	static float ez_sum = 0;


	//弧度值陀螺仪数据
	float gx_rad, gy_rad, gz_rad;

	//四元数转余弦矩阵中间变量
	float g1, g2, g3, g4, g5;


	float ax = g_imu_struct.ax_raw;
	float ay = g_imu_struct.ay_raw;
	float az = g_imu_struct.az_raw;
	float mx = g_imu_struct.mx_raw;
	float my = g_imu_struct.my_raw;
	float mz = g_imu_struct.mz_raw;

	float q0 = g_imu_struct.quat.q0;
	float q1 = g_imu_struct.quat.q1;
	float q2 = g_imu_struct.quat.q2;
	float q3 = g_imu_struct.quat.q3;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
	//更新数据

	//零漂补偿并弧度制陀螺仪数据
	gx_rad = (g_imu_struct.gx_raw - g_imu_struct.gx_offset) * BMI088_GYRO_2000_SEN;
	gy_rad = (g_imu_struct.gy_raw - g_imu_struct.gy_offset) * BMI088_GYRO_2000_SEN;
	gz_rad = (g_imu_struct.gz_raw - g_imu_struct.gz_offset) * BMI088_GYRO_2000_SEN;

	//加速度归一化
	norm_temp = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm_temp;
	ay = ay * norm_temp;
	az = az * norm_temp;
	//用陀螺仪的数据计算物体坐标系重力分量
    vx = 2.0f * (q1q3 - q0q2);
    vy = 2.0f * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
#ifdef IST8310_IN_WORK
	//磁力计归一化
    norm_temp = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm_temp;
    my = my * norm_temp;
    mz = mz * norm_temp;


	//用陀螺仪的数据计算物体坐标系磁力分量
    hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
    hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
    hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;


    wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
    wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
    wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
#else
    mx = 0;
    my = 0;
    mz = 0;
#endif
    //求姿态误差
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);


	//误差积分
	ex_sum += g_imu_struct.Ki * dt * ex;
	ey_sum += g_imu_struct.Ki * dt * ey;
	ez_sum += g_imu_struct.Ki * dt * ez;
	//互补滤波
	gx_rad = gx_rad + g_imu_struct.Kp * ex + ex_sum;
	gy_rad = gy_rad + g_imu_struct.Kp * ey + ey_sum;
	gz_rad = gz_rad + g_imu_struct.Kp * ez + ez_sum;

//    now_update = HAL_GetTick(); //ms
//    halfperiod = ((float)(now_update - last_update) / 2000.0f);
//    last_update = now_update;

	//解四元数微分方程更新四元数
	q0 = q0 +  0.5 * dt *(-gx_rad * q1 - gy_rad * q2 - gz_rad * q3);
	q1 = q1 +  0.5 * dt *( gx_rad * q0 - gy_rad * q3 + gz_rad * q2);
	q2 = q2 +  0.5 * dt *( gx_rad * q3 + gy_rad * q0 - gz_rad * q1);
	q3 = q3 +  0.5 * dt *(-gx_rad * q2 + gy_rad * q1 + gz_rad * q0);
	//四元数归一化
	norm_temp = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm_temp;
	q1 = q1 * norm_temp;
	q2 = q2 * norm_temp;
	q3 = q3 * norm_temp;
	//将更新后的四元数存到到g_imu_struct中
	g_imu_struct.quat.q0 = q0;
	g_imu_struct.quat.q1 = q1;
	g_imu_struct.quat.q2 = q2;
	g_imu_struct.quat.q3 = q3;
	//四元数转旋转矩阵
	g1 = 2.0f * (q1 * q3 - q0 * q2);
	g2 = 2.0f * (q0 * q1 + q2 * q3);
	g3 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	g4 = 2.0f * (q1 * q2 + q0 * q3);
	g5 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;

	*roll = atan2f(g2,g3) * 57.29578;
	*pitch = -asinf(g1) * 57.29578;
	*yaw = atan2f(g4, g5) * 57.29578;

}

static void IMU_Temperature_Compensate(void)
{
	float real_temp;
	float temp_pid_out;
	uint16_t temp_pwm;
	static PID_Typedef s_temp_pidstruct = {
			.param.Kp = 150,
			.param.Ki = 0.1,
			.limit.max_err_input = 100,
			.limit.max_i_out = IMU_TEMP_PWM_MAX - 500,
			.output.i_out = 200,
			.limit.max_total_out = IMU_TEMP_PWM_MAX
	};

	real_temp = g_imu_struct.temperature;

	if (real_temp < 38.5f)
	{
		//温度低于36度，满功率加热
		if (real_temp<36)
		{
			__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, IMU_TEMP_PWM_MAX);
			return ;
		}
		//温度高于36度但低于38.5度，半功率加热
		__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, IMU_TEMP_PWM_MAX / 2);
		return ;
	}

	//温度高于38.5度，用pid控制温度
	temp_pid_out = Pid_Calculate(&s_temp_pidstruct, real_temp, IMU_MAX_TEMP_SET);
	if (temp_pid_out < 0)
	{
		temp_pwm = 0;
	}
	else
	{
		temp_pwm =  (uint16_t)temp_pid_out;
	}
	__HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, temp_pwm);
}





