

#include "MyProject.h"
#include "as5600.h"
#include "BLDCMotor.h"
#include "MagneticSensor.h"
/************************************************
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/



#define  AS5600_Address  0x36
#define  RAW_Angle_Hi    0x0C   //V2.1.1 bugfix
//#define  RAW_Angle_Lo    0x0D
#define  AS5600_CPR      4096
#define I2C_getRawCount  bsp_as5600GetRawAngle
#define I2C_getRawCount2  bsp_as5600GetRawAngle2


static float angel_his_1 = 0;
static float angel_his_2 = 0;
static int lock_flag_iic_1 = 0;
static int lock_flag_iic_2 = 0;


static int lock_iic(int *lock_flag)
{
	if (*lock_flag == 1) {
		return -1;
	}
	*lock_flag = 1;
	return 0;
}


static int looc_iic(int *lock_flag)
{
	*lock_flag = 0;
	return 0;
}


void MagneticSensor_Init(MOTOR_FOC *motor)
{
	motor->cpr=AS5600_CPR;
	if(motor->motor_name == MOTOR_1)
		motor->angle_data_prev = I2C_getRawCount();
	if(motor->motor_name == MOTOR_2)
		motor->angle_data_prev = I2C_getRawCount2();

	motor->full_rotation_offset = 0;
	motor->velocity_calc_timestamp=0;
}


float getAngle(MOTOR_FOC *motor)
{
	float angle_data,d_angle;
	float angle_out = 0;

	if(motor->motor_name == MOTOR_1) {
		if (lock_iic(&lock_flag_iic_1) == -1) {
			return angel_his_1;
		}
		angle_data = I2C_getRawCount();
	} else if(motor->motor_name == MOTOR_2) {
		if (lock_iic(&lock_flag_iic_2) == -1) {
			return angel_his_2;
		}
		angle_data = I2C_getRawCount2();
	} else {
		return 0;
	}

	// tracking the number of rotations
	// in order to expand angle range form [0,2PI] to basically infinity
	d_angle = angle_data - motor->angle_data_prev;

	// if overflow happened track it as full rotation
	if(fabs(d_angle) > (0.8*motor->cpr) ) motor->full_rotation_offset += d_angle > 0 ? -_2PI : _2PI;

	// save the current angle value for the next steps
	// in order to know if overflow happened
	motor->angle_data_prev = angle_data;

	// return the full angle
	// (number of full rotations)*2PI + current sensor angle
	angle_out =  (motor->full_rotation_offset + ( angle_data / (float)motor->cpr) * _2PI) ;

	if(motor->motor_name == MOTOR_1) {
		angel_his_1 = angle_out;
		looc_iic(&lock_flag_iic_1);
	} else if(motor->motor_name == MOTOR_2) {
		angel_his_2 = angle_out;
		looc_iic(&lock_flag_iic_2);
	}

	return angle_out;
}


float get_velocity(MOTOR_FOC *motor)
{
	return motor->tim_velocity_data;
}


void tim_velocity(MOTOR_FOC *motor)
{
	float Ts, angle_c, vel;
	Ts = 0.005; // 200Hz=0.005秒

	// current angle
	angle_c = getAngle(motor);
	// velocity calculation
	vel = (angle_c - motor->angle_prev)/Ts;

	// save variables for future pass
	motor->angle_prev = angle_c;
	motor->tim_velocity_data =  vel;
}

