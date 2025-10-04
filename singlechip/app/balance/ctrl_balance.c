#include "Kalman.h"
#include "main.h"
#include "ctrl_balance_pid.h"
#include "usart_device.h"

typedef struct
{
    float target_speed;         // 速度环目标速度
    float his_speed;
    float out_Speed;
    float line_speed_right;   // 轮子线速度
    float line_speed_left;    // 轮子线速度

    float target_angle; // 期望角度
    float angle;        // 俯仰角
    float accelerat;    // 加速度
} BALANCE_CTX;

BALANCE_CTX balance_ctx = {0};


// 获取轮子线速度
void ctrl_balance_update_speed(float speed_left, float speed_right)
{
    balance_ctx.line_speed_right = speed_right;
    balance_ctx.line_speed_left = speed_left;
}

// 获取角度
void ctrl_balance_update_angle(float angle, float gyro)
{
    balance_ctx.angle = angle;
    balance_ctx.accelerat = -gyro;
}


void ctrl_balance_speed_out(float *out_Speed)
{
    *out_Speed = balance_ctx.out_Speed;
}

// 50hz执行
void ctrl_balance_vertical_task(void)
{
    float feedback_Speed = 0;
    float vertical_pid_out = 0;
    float speed_pid_out = 0;

	feedback_Speed = (balance_ctx.line_speed_right + balance_ctx.line_speed_left) * 0.5f;
	if (feedback_Speed > 80 || feedback_Speed < -80) {
		feedback_Speed = balance_ctx.his_speed;
	} else {
		balance_ctx.his_speed = feedback_Speed;
	}

    // 对速度做滤波处理
	// feedback_Speed = KalmanFilter(&kfp1, feedback_Speed);

	// 判断当前角度与直立状态的机械平衡角度差值在±10°范围内
	if (fabs(balance_ctx.angle) <= fabs(balance_ctx.target_angle) + 35) {

        // 直立环PID 主要使用PD控制
		vertical_pid_out = pid_vertical(balance_ctx.target_angle, balance_ctx.angle, balance_ctx.accelerat);

        // 速度环
		speed_pid_out = pid_speed(balance_ctx.target_speed, feedback_Speed);

        // 最终输出由直立环输出减去速度环输出
		balance_ctx.out_Speed = vertical_pid_out - speed_pid_out;
	}
	else{
		balance_ctx.out_Speed = 0;
	}

    sprintf((char*)send_buf, "D:%f,%f,%f,%f,%f,%f,%f,%f\r\n",
    balance_ctx.line_speed_right,
    balance_ctx.line_speed_left,
    balance_ctx.target_angle,
    balance_ctx.angle,
    vertical_pid_out,
    balance_ctx.target_speed,
    feedback_Speed,
    speed_pid_out
    );
    usart2_driver_Transmit(send_buf,sizeof(send_buf));
}


void ctrl_balance_init(void)
{
    Kalman1_Init();

    balance_ctx.target_speed = 0;
    balance_ctx.target_angle = -3.5;
}

void set_angle(int angle)
{
    balance_ctx.target_angle = -angle*0.01;
}

