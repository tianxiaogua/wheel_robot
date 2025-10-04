#include "ctrl_balance_pid.h"

#ifdef DEBUG_PID
float KP    = (5.00f);
float KD    = (0.01f);
float S_KP  = (3.52);
float S_KI  = (0.01);
#else
#define KP    (5.00f)
#define KD    (0.01f)
#define S_KP  (3.52)
#define S_KI  (0.01)
#endif

float bias_last = 0;
float sum_error;

float pid_vertical(float target, float feedback, float gyro)
{
    float output;
    float bias;
    float dif = 0;

    /** 计算目标值和反馈值的差值*/
    bias = target - feedback;

    /** 计算本次误差和上次误差的插值得到偏差数*/
    dif = bias - bias_last;

    /** 计算最终的PID输出*/
    output = KP * bias + KD * gyro;

    /** 更新历史误差值*/
    bias_last = bias;

	return output;
}


float pid_speed(float target, float feedback)
{
    float output;
    float bias;

    /** 计算目标值和反馈值的差值*/
    bias = target - feedback;

    /** 计算积分*/
    sum_error += bias;

    /** 计算最终的PID输出*/
	output = S_KP * bias + S_KI * sum_error;

	return output;
}

#ifdef DEBUG_PID
void set_PID(int _KP, int _KD, int _S_KP, int _S_KI)
{
	KP   = _KP*0.01f;
	KD   = _KD*0.001f;
	S_KP = _S_KP*0.01f;
    S_KI = _S_KI*0.001f;
    printf("D:%f,%f,%f,%f\r\n", KP, KD, S_KP, S_KI);
}
#endif

