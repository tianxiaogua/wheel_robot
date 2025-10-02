#include "ctrl_balance_pid.h"

PID_PARA PID_vertical; // 直立环PID参数
PID_PARA PID_speed; // 速度环PID参数


/*******************************************************************************
 * @file   pid.c
 * @brief  经典PID
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
float KP = 1.63f ; // 1.3 1.2 1.0 1.4 1.55 1.8 1.8 1.66
float KD = 0.1f	; // 0.2 0.5 1.3 1.0 0.85 0.88   1 1.15
float dif = 0;
float bias_last = 0;
float pid_vertical(float target, float feedback, float gyro)
{
    float output;
    float bias;
    /** 计算目标值和反馈值的差值*/
    bias = target - feedback;

    /** 计算本次误差和上次误差的插值得到偏差数*/
    dif = bias - bias_last;
    // printf("%.3f,",PID->dif);

    /** 计算最终的PID输出*/
    output = KP * bias + KD * gyro;
    /** 更新历史误差值*/
    bias_last = bias;
    ///printf("D:%.3f,%.3f,%.3f,%.3f,%.3f,", target, feedback, KP*bias, KD*dif, output);
	return output;
}

float S_KP = 1.41; // 0.6
float S_KI = 0.00;
float sum_error;
float pid_speed(float target, float feedback)
{
    float output;
    float bias;
    /** 计算目标值和反馈值的差值*/
    bias = target - feedback;
    sum_error += bias;
    /** 计算最终的PID输出*/
	output = S_KP*bias + (S_KP/200)*sum_error;

    /** 更新历史误差值*/
    //printf("%.3f,%.3f,%.3f,%.3f\n", feedback,S_KP*bias, S_KI*sum_error, output);
    //printf("%.3f\n",output);
	return output;
}

void set_PID(float _KP, float _KD, float _S_KP)
{
	KP   = _KP;
	KD   = _KD;
	S_KP = _S_KP;
}


