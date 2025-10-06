#ifndef BLDCMotor_H
#define BLDCMotor_H

#include "foc_utils.h"


/******************************************************************************/
/**
 *  Direction structure
 */
typedef enum
{
    CW      = 1,  //clockwise
    CCW     = -1, // counter clockwise
    UNKNOWN = 0   //not yet known or invalid state
} Direction;

/******************************************************************************/
#define MOTOR_1 1
#define MOTOR_2 2

typedef struct
{
	/* data */
	float pid_vel_P, pid_ang_P;
	float pid_vel_I, pid_ang_D;
	float integral_vel_prev;
	float error_vel_prev, error_ang_prev;
	float output_vel_ramp;
	float output_vel_prev;
	unsigned long pid_vel_timestamp, pid_ang_timestamp;
}PID;

typedef struct
{
    /* data */
    /******************************************************************************/
    float shaft_angle;        // 电机累计转过的轴角度
    float electrical_angle;   // 电机电器角度
    float shaft_velocity;     // 轴速度  弧度/S
    float current_sp;         // 电流环，速度PID输出
    float shaft_velocity_sp;  // 速度环目标值
    float shaft_angle_sp;     //
    DQVoltage_s voltage;
    DQCurrent_s current;

    // TorqueControlType torque_controller;
    // MotionControlType controller;

    float sensor_offset;
    float zero_electric_angle;

    float Ta;
    float Tb;
    float Tc;
    /******************************************************************************/
}FOC;

typedef struct
{
    /* data */
    int motor_name; // motor
    long sensor_direction;
    float voltage_power_supply;
    float voltage_limit;
    float voltage_sensor_align;
    int  pole_pairs;
    unsigned long open_loop_timestamp;
    float velocity_limit;

    long  cpr;
    float full_rotation_offset;
    long  angle_data_prev;
    unsigned long velocity_calc_timestamp;
    float angle_prev;

    float tim_velocity_data; // 中断中计算得到的轴速度

    PID pid;

    FOC foc;

}MOTOR_FOC;

extern MOTOR_FOC motor_1;
extern MOTOR_FOC motor_2;

void foc_init(MOTOR_FOC *motor);
void foc_loop_handle(MOTOR_FOC *motor, float target_speed);

#endif
