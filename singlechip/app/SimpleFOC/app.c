#include "app.h"

#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"


#include "main.h"

#include "stdio.h"
#include "nvic_device.h"
#include "as5600.h"
//#include "STM32bsp.h"
#include "pid.h"
#include "MagneticSensor.h"
#include "pwm_device.h"
#include "usart_device.h"
#include "stdlib.h"
#include "stdio.h"
#include<string.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"

#include "Kalman.h"

#include "rc_filter.h"

#include "ctrl_balance.h"

#include <stdlib.h>
#include "ctrl_balance_pid.h"

/*欧拉角euler angle 角度制 0-360°*/
typedef struct
{
	float pitch;
	float roll; // 角度
	float yaw;

	short gyrox; // 加速度
	short gyroy;
	short gyroz;
}EULER_ANGLE;
EULER_ANGLE euler_angle;

typedef struct
{
	uint32_t times_falg;
	int32_t interupt_200hz_flag;
} TIME_INTERUPT_CTX;
TIME_INTERUPT_CTX g_time_interupt_ctx = {0};


#ifdef DEBUG_PID
void test_motor(void)
{
	float test_angle = 0;

	while (1)
	{
		// HAL_Delay(1);
		test_angle += 0.01;
		if (test_angle > _2PI) {
			test_angle = 0;
		}
		setPhaseVoltage(&motor_1, 20, 0, test_angle);
		setPhaseVoltage(&motor_2, 20, 0, test_angle);

		sprintf((char*)send_buf, "D:%f,%f,%f,%f,%f,%f,%f,%f\r\n",
		test_angle,
		euler_angle.pitch,
		motor_1.foc.shaft_velocity,
		motor_2.foc.shaft_velocity,
		motor_1.foc.shaft_angle,
		motor_2.foc.shaft_angle,
		motor_1.foc.electrical_angle,
		motor_2.foc.electrical_angle
		);
		usart_driver_Transmit(send_buf,sizeof(send_buf));
	}
}
#endif

static void simpleFOC_init(void)
{
	motor_1.motor_name = MOTOR_1;
	motor_1.voltage_power_supply=12;   // FOC power
	motor_1.pole_pairs=7;              // Number of motor poles
	motor_1.voltage_limit=6;           // Phase voltage limitation
	motor_1.velocity_limit=20;         //rad/s angleOpenloop() and PID_angle() use it
	motor_1.voltage_sensor_align=2.5;  //

	motor_2.motor_name = MOTOR_2;
	motor_2.voltage_power_supply=12;   // FOC power
	motor_2.pole_pairs=7;              // Number of motor poles
	motor_2.voltage_limit=6;           // Phase voltage limitation
	motor_2.velocity_limit=20;         //rad/s angleOpenloop() and PID_angle() use it
	motor_2.voltage_sensor_align=2.5;  //

	// torque_controller=Type_voltage;  //
	// controller=Type_velocity;  //Type_angle; //Type_torque;    //

	MagneticSensor_Init(&motor_1);     //AS5600 or TLE5012B
	MagneticSensor_Init(&motor_2);     //AS5600 or TLE5012B
	Motor_init(&motor_1);
	Motor_init(&motor_2);
	Motor_initFOC(&motor_1);
	Motor_initFOC(&motor_2);
	PID_init(&motor_1);                //PID init
	PID_init(&motor_2);                //PID init
	printf("Motor ready.\r\n");
}


static void nvic_200hz_callback(void)
{
	g_time_interupt_ctx.times_falg++;
    if(g_time_interupt_ctx.times_falg>=1000){
		g_time_interupt_ctx.times_falg = 0;
    }

	if (g_time_interupt_ctx.times_falg % 5 == 0 || g_time_interupt_ctx.times_falg == 0) { // 200HZ
		g_time_interupt_ctx.interupt_200hz_flag = 1;
		tim_velocity(&motor_1);
		tim_velocity(&motor_2);
	}
}


static uint8_t get_euler_angle(void)
{
    /*陀螺仪获取角度部分*/
	int recv = mpu_dmp_get_data(&euler_angle.pitch, &euler_angle.roll,	&euler_angle.yaw);
	// if(	recv != 0) {
	// 	printf("angle error recv:%d\r\n",recv);
	// }

	MPU_Get_Gyroscope(&euler_angle.gyrox,&euler_angle.gyroy,&euler_angle.gyroz);	//陀螺仪
	// printf("D:%f\n", euler_angle.gyrox / 16.4 + 1.05);
    return recv;
}

#ifdef DEBUG_PID
int a = 0, b=0, c=0, d=0,f=0;
static void app_get_buf_data(uint8_t *data)
{
	unsigned char buf[32] = {0};

	strcpy((char *)buf, (char *)data);
	int temp_data = atoi((char *)&buf[2]);
	if (buf[0] == 'P') {
		a  = temp_data;
		printf("get P %d\r\n", temp_data);
	} else if (buf[0] == 'D') {
		printf("get D %d\r\n", temp_data);
		b = temp_data;
	} else if (buf[0] == 'S') {
		printf("get s %d\r\n", temp_data);
		c = temp_data;
	} else if (buf[0] == 'I') {
		printf("get s %d\r\n", temp_data);
		d = temp_data;

	} else if (buf[0] == 'A') {
		printf("get f %d\r\n", temp_data);
		f = temp_data;
	} else {

	}

	set_PID(a, b, c, d);
	set_angle(f);
}
#endif

void app(void)
{
	float speed_Ma = 0;
 	float speed_Mb = 0;
	float out_Speed = 0;

	// test_motor();
	while(1) {
#ifdef DEBUG_PID
		if (Rx2_Flag == 1) {
			Rx2_Flag = 0;
			printf("get it\n\n");
			app_get_buf_data(Rx2_Buf);
		}
#endif

		get_euler_angle();
		ctrl_balance_update_speed(motor_1.foc.shaft_velocity, -motor_2.foc.shaft_velocity);
		ctrl_balance_update_angle(euler_angle.pitch, euler_angle.gyroy);
		ctrl_balance_vertical_task();
		ctrl_balance_speed_out(&out_Speed);
		speed_Ma = out_Speed;
		speed_Mb = -out_Speed;

		loopFOC(&motor_1, speed_Ma);
		loopFOC(&motor_2, speed_Mb);

		// sprintf((char*)send_buf, "D:%f,%f,%f,%f,%f,%f,%f,%f\r\n",
		// (float)euler_angle.gyroy,
		// euler_angle.pitch,
		// motor_1.foc.shaft_velocity,
		// motor_2.foc.shaft_velocity,
		// motor_1.foc.shaft_angle,
		// motor_2.foc.shaft_angle,
		// motor_1.foc.electrical_angle,
		// motor_2.foc.electrical_angle
		// );
		// usart2_driver_Transmit(send_buf,sizeof(send_buf));
	}
}





void app_init(void)
{
    int recv = bsp_as5600Init();
	if(recv != 0){
		printf("bsp_as5600Init error!!\r\n");
		bsp_as5600Init();
		return;
	}
	recv = bsp_as5600Init2();
	if(recv != 0){
		printf("bsp_as5600Init2 error!!\r\n");
		bsp_as5600Init2();
		return;
	}
	HAL_GPIO_WritePin(DRV_EN_GPIO_Port, DRV_EN_Pin, GPIO_PIN_SET); //
	printf("%.2f %.2f\r\n",get_angle(), get_angle2());

	// 对MPU6050进行测试
    if(MPU_Init()){ //初始化MPU6050
        printf("MPU_Init\r\n");
    }

    while(mpu_dmp_init()) {//初始化 MPU6050的DMP
        printf("mpu_dmp_init error\r\n");
        HAL_Delay(10);
		return;
    }

    printf("init mpu 6050 done\r\n");

	nvic_register_callback(nvic_200hz_callback);
	ctrl_balance_init();
	init_PWM_motor();
	start_interrupt();
	simpleFOC_init();
	init_usart_interupt();

	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	app();
}
