#include "nvic_device.h"
//#include "foc.h"
#include "as5600.h"
#include "stdio.h"
#include "usart_device.h"
#include "adc_device.h"
#include "MyProject.h"
#include "BLDCMotor.h"
#include "FOCMotor.h"
#include "BLDCMotor.h"
#include "MagneticSensor.h"
#include "app.h"
/*******************************************************************************
 * @file   : vnic_device.c
 * @brief  : 启动中断
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
void start_interrupt(void)
{

#if  FOC_CURRENT_LOOP
  HAL_TIM_Base_Start_IT(&htim7); // main loop interrupt
  HAL_TIM_Base_Start_IT(&htim1); // gat adc timer
  HAL_TIM_Base_Start_IT(&htim5); // main loop interrupt
  HAL_TIM_Base_Start_IT(&htim2); // speed feedback
#else
  HAL_TIM_Base_Start_IT(&htim2); // speed feedback  20Hz
#endif
}


/*******************************************************************************
 * @file   : vnic_device.c
 * @brief  : 设置定时器的中断2ms一次，定时器中断函数 在50ms 20Hz一次的中断内 通过计数的方式分频出50、100、200、500ms
定时时间： Time = (ARR+1) * (PSC+1) / Tclk = （重装载值+1）（预分频系数+1）/ 时钟频率 = （7199+1） （9999+1）/ 72 000 000 = 1s
 * @author : tianxiaohua
 * @date   : 2023-04
 ******************************************************************************/
uint32_t time_flag = 0;
float history_L_Mspeed = 0;
float history_R_Mspeed = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == &htim2)  // htim2 被设置为1毫秒中断一次
  {
    time_flag++;
    if(time_flag>=1000){
      time_flag = 0;
    }

    if (time_flag%20 == 0) { //50Hz
      sand_back_speed(get_velocity(&motor_1), get_velocity(&motor_2));
    }
    if (time_flag%2 == 0) { // 500Hz
      tim_velocity(&motor_1); // 在中断中执行计算速度
      tim_velocity(&motor_2); // 在中断中执行计算速度
    }
  }
}
