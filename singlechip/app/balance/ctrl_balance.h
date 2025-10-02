#ifndef CTRL_BALANCE_H
#define CTRL_BALANCE_H


// 初始化
void ctrl_balance_init(void);

// 50hz执行
void ctrl_balance_vertical_task(void);

// 获取角度
void ctrl_balance_update_angle(float angle, float gyro);

// 获取轮子线速度
void ctrl_balance_update_speed(float speed_left, float speed_right);

void ctrl_balance_speed_out(float *out_Speed);


#endif


