
#include "BLDCmotor.h"
#include "FOCMotor.h"
#include "MagneticSensor.h"
#include "main.h"
#include "stdio.h"


float y_vel_prev=0;
float y_vel_prev2=0;

// 一阶低通滤波器（Low Pass Filter, LPF），专门用来对小车速度信号进行平滑处理。
float LPF_velocity(float x)
{
	float y = 0.9f * y_vel_prev + 0.1f * x;

	y_vel_prev=y;

	return y;
}

float LPF_velocity2(float x)
{
	float y = 0.9f * y_vel_prev2 + 0.1f * x;

	y_vel_prev2=y;

	return y;
}


// 轴角计算
float shaftAngle(MOTOR_FOC *motor)
{
  // if no sensor linked return previous value ( for open loop )
  //if(!sensor) return shaft_angle;
  return motor->sensor_direction*getAngle(motor) - motor->foc.sensor_offset;
}


// 轴速计算
float shaftVelocity(MOTOR_FOC *motor)
{
  // if no sensor linked return previous value ( for open loop )
  if(motor->motor_name == MOTOR_1) {
    return motor->sensor_direction*LPF_velocity(get_velocity(motor));
  }

  if(motor->motor_name == MOTOR_2) {
    return motor->sensor_direction*LPF_velocity2(get_velocity(motor)); // motor->tim_velocity_data从中断中计算得到的轴速度
  }


}


// 电器角度
float electricalAngle(MOTOR_FOC *motor)
{
  return _normalizeAngle((motor->foc.shaft_angle + motor->foc.sensor_offset) * motor->pole_pairs - motor->foc.zero_electric_angle);
}


