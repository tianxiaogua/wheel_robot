#ifndef RC_H
#define RC_H

#include "main.h"

/*******************************************************************************
 * @brief  : 一阶滤波
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
float first_order_filter(float data);

/*******************************************************************************
 * @brief  : 一阶滤波
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
float first_order_filter_speed_out(float data);

float first_order_filter_speed(float data, float his_data);

#endif

