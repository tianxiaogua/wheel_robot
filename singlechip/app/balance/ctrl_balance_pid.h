#ifndef balance_pid_h
#define balance_pid_h

#include "stdio.h"
#include<stdlib.h>
#include<math.h>
/* PID参数*/
typedef struct
{
    /* data */
    float KP;
    float KI;
    float KD;
    float bias;            // 计算值和实际值的偏差
    float sum_error;       // 积分偏差
    float dif;             // 微分differential
    float bias_last;             // 微分differential
    float limit_bias;      // 对误差的限幅limit
    float limit_sum_error; // 对积分限幅
    float limit_out;       // 对输出限幅
    float pid_out;             // 输出
}PID_PARA;

extern PID_PARA PID_vertical; // 直立环PID参数
extern PID_PARA PID_speed; // 速度环PID参数

/*******************************************************************************
 * @file   pid.c
 * @brief  初始化PID参数
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
void init_pid(void);

/*******************************************************************************
 * @file   pid.c
 * @brief  经典PID
 * @author Tianxiaogua
 * @date   2023-04
 ******************************************************************************/
//float pid_vertical(float target, float feedback);
float pid_vertical(float target, float feedback, float gyro);
float pid_speed(float target, float feedback);

void set_PID(float _KP, float _KD, float _S_KP);

#endif
