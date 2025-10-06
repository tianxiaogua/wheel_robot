#include "stdio.h"
#include "pwm_device.h"
#include "MagneticSensor.h"
#include "pid.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "app.h"

/************************************************
main中调用的接口函数都在当前文件中
=================================================
本程序仅供学习，引用代码请标明出处
使用教程：https://blog.csdn.net/loop222/article/details/120471390
创建日期：20210925
作    者：loop222 @郑州
************************************************/

#define delay_ms HAL_Delay

/******************************************************************************/
extern float target;
/******************************************************************************/
// long sensor_direction;
// float voltage_power_supply;
// float voltage_limit;
// float voltage_sensor_align;
// int  pole_pairs;
// unsigned long open_loop_timestamp;
// float velocity_limit;
MOTOR_FOC motor_1;
MOTOR_FOC motor_2;


void Motor_init(MOTOR_FOC *motor)
{
	printf("MOT: Init\r\n");

//	new_voltage_limit = current_limit * phase_resistance;
//	voltage_limit = new_voltage_limit < voltage_limit ? new_voltage_limit : voltage_limit;
	if(motor->voltage_sensor_align > motor->voltage_limit) {
		motor->voltage_sensor_align = motor->voltage_limit;
	}

	motor->sensor_direction=UNKNOWN;

//	M1_Enable;
	printf("MOT: Enable driver.\r\n");
}


// SVPWM转换
void setPhaseVoltage(MOTOR_FOC *motor ,float Uq, float Ud, float angle_el)
{
	float Uout = 0;
	unsigned int sector = 0;
	float T0,T1,T2;
	float Ta,Tb,Tc;

	if(Ud) // only if Ud and Uq set
	{// _sqrt is an approx of sqrt (3-4% error)
		Uout = _sqrt(Ud*Ud + Uq*Uq) / motor->voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
	}
	else
	{// only Uq available - no need for atan2 and sqrt
		Uout = Uq / motor->voltage_power_supply;
		// angle normalisation in between 0 and 2pi
		// only necessary if using _sin and _cos - approximation functions
		angle_el = _normalizeAngle(angle_el + _PI_2);
	}
	if(Uout> 0.577)Uout= 0.577;
	if(Uout<-0.577)Uout=-0.577;

	sector = (angle_el / _PI_3) + 1;
	T1 = _SQRT3*_sin(sector*_PI_3 - angle_el) * Uout;
	T2 = _SQRT3*_sin(angle_el - (sector-1.0)*_PI_3) * Uout;
	T0 = 1 - T1 - T2;

	// calculate the duty cycles(times)
	switch(sector)
	{
		case 1:
			Ta = T1 + T2 + T0/2;
			Tb = T2 + T0/2;
			Tc = T0/2;
			break;
		case 2:
			Ta = T1 +  T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T0/2;
			break;
		case 3:
			Ta = T0/2;
			Tb = T1 + T2 + T0/2;
			Tc = T2 + T0/2;
			break;
		case 4:
			Ta = T0/2;
			Tb = T1+ T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 5:
			Ta = T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T2 + T0/2;
			break;
		case 6:
			Ta = T1 + T2 + T0/2;
			Tb = T0/2;
			Tc = T1 + T0/2;
			break;
		default:  // possible error state
			Ta = 0;
			Tb = 0;
			Tc = 0;
	}
	Ta = Ta*3600;
	Tb = Tb*3600;
	Tc = Tc*3600;
	if(motor->motor_name == MOTOR_1){
		set_pwm_channel_1(Tb);
		set_pwm_channel_2(Tc);
		set_pwm_channel_3(Ta);

		motor->foc.Ta = Ta;
		motor->foc.Tb = Tb;
		motor->foc.Tc = Tc;
	}
	if(motor->motor_name == MOTOR_2){
		set_pwm_channel_1_2(Tb);
		set_pwm_channel_2_2(Tc);
		set_pwm_channel_3_2(Ta);
		motor->foc.Ta = Ta;
		motor->foc.Tb = Tb;
		motor->foc.Tc = Tc;
	}
}


void foc_init(MOTOR_FOC *motor)
{
	motor_1.pole_pairs=  7;
	motor_1.sensor_direction=CW;
	motor_1.foc.zero_electric_angle = 3.3778;

	motor_2.pole_pairs=  7;
	motor_2.sensor_direction=CW;
	motor_2.foc.zero_electric_angle = 2.1783;
	setPhaseVoltage(motor, 0, 0, 0);
	delay_ms(200);

	//added the shaft_angle update
	motor->angle_prev = getAngle(motor);  //getVelocity(),make sure velocity=0 after power on
	delay_ms(5);
	motor->foc.shaft_velocity = shaftVelocity(motor);  //必须调用一次，进入主循环后速度为0
	printf("shaft_velocity %f\r\n",motor->foc.shaft_velocity);
	delay_ms(5);
	motor->foc.shaft_angle = shaftAngle(motor);// shaft angle

	// if(controller==Type_angle)
	// 	target = shaft_angle;//角度模式，以当前的角度为目标角度，进入主循环后电机静止

	delay_ms(200);
}


// 电流环
void electric_current_loop(MOTOR_FOC *motor, float new_target)
{
	motor->foc.shaft_velocity = shaftVelocity(motor);
	// velocity set point
	motor->foc.shaft_velocity_sp = new_target;
	// calculate the torque command
	motor->foc.current_sp = PID_velocity(motor, motor->foc.shaft_velocity_sp - motor->foc.shaft_velocity); // if current/foc_current torque control
	// if torque controlled through voltage control

	motor->foc.voltage.q = motor->foc.current_sp;  // use voltage if phase-resistance not provided
	motor->foc.voltage.d = 0;
}


// FOC算法 包括速度环、角度环和电流环，目前只有速度环
void foc_loop_handle(MOTOR_FOC *motor, float target_speed)
{
	motor->foc.shaft_velocity = shaftVelocity(motor);
	// velocity set point
	motor->foc.shaft_velocity_sp = target_speed;
	// calculate the torque command
	motor->foc.current_sp = PID_velocity(motor, motor->foc.shaft_velocity_sp - motor->foc.shaft_velocity); // if current/foc_current torque control
	// if torque controlled through voltage control

	motor->foc.voltage.q = motor->foc.current_sp;  // use voltage if phase-resistance not provided
	motor->foc.voltage.d = 0;

	motor->foc.shaft_angle = shaftAngle(motor);// shaft angle
	motor->foc.electrical_angle = electricalAngle(motor);// electrical angle - need shaftAngle to be called first

  	setPhaseVoltage(motor, motor->foc.voltage.q, motor->foc.voltage.d, motor->foc.electrical_angle);
}