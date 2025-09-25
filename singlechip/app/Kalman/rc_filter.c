#include "rc_filter.h"

float his_data_order_filter = 0;
#define COEFFICIENT 0.8f // 一阶滤波 对速度进行滤波；
float first_order_filter(float data)
{
  float out_data = data*COEFFICIENT + his_data_order_filter*(1-COEFFICIENT);
  return out_data;
}

float his_data_speed_out = 0;
#define COEFFICIENT_ 0.5f // 一阶滤波 对速度进行滤波；
float first_order_filter_speed_out(float data)
{
  float out_data = data*COEFFICIENT_ + his_data_speed_out*(1-COEFFICIENT_);
  return out_data;
}

float first_order_filter_speed(float data, float his_data)
{
  float out_data = data*COEFFICIENT + his_data*(1-COEFFICIENT);
  return out_data;
}

