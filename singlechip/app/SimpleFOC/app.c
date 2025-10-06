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

#define CAR_PARM_WHEEL_RADI   (0.03f) // 车轮半径，单位米
#define CAR_PARM_WHEEL_RTEAD  (0.12f) // 车轮轮距，单位米
#define M_PI	              (3.14159265358979323846)


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



/**
 * @brief 计算差速小车的整体速度（线速度 m/s，角速度 deg/s）
 * @param wl       左轮角速度 (rad/s)
 * @param wr       右轮角速度 (rad/s)
 * @param wheel_r  轮子半径 (m)
 * @param wheel_L  两轮间距 (m)
 * @param v        输出参数：小车线速度 (m/s)
 * @param w_deg    输出参数：小车角速度 (deg/s)
 */
void app_calc_robot_velocity(float wl, float wr, float wheel_r, float wheel_L,
                             float* v, float* w_deg)
{
    // 左右轮线速度
    float vl = wl * wheel_r; // m/s
    float vr = wr * wheel_r; // m/s

    // 小车线速度
    *v = (vr + vl) / 2.0f;

    // 小车角速度（先 rad/s 再转 deg/s）
    float w_rad = (vr - vl) / wheel_L;
    *w_deg = w_rad * 180.0f / M_PI;
}


/**
 * @brief 根据小车线速度和角速度计算左右轮线速度
 * @param v         小车线速度 (m/s)
 * @param w         小车角速度 (rad/s)
 * @param wheel_L   轮距 (m)
 * @param out_vl    输出左轮线速度 (m/s)
 * @param out_vr    输出右轮线速度 (m/s)
 */
void app_diff_drive_calc_wheel_vel(float v, float w, float wheel_L, float* out_vl, float* out_vr)
{
    *out_vr = v + w * wheel_L / 2.0f;
    *out_vl = v - w * wheel_L / 2.0f;
}


/**
 * @brief   将轮子角速度转换为线速度
 * @param   omega   角速度 (rad/s)
 * @param   radius  轮子半径 (m)
 * @return  线速度 (m/s)
 */
static float app_wheel_angular_to_linear(float omega, float radius)
{
    return omega * radius;
}


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
	// Motor_init(&motor_1);
	// Motor_init(&motor_2);
	foc_init(&motor_1);
	foc_init(&motor_2);
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
	MPU_Get_Gyroscope(&euler_angle.gyrox,&euler_angle.gyroy,&euler_angle.gyroz);	//陀螺仪
    return recv;
}


/**
 * @brief 将4字节数组转换为有符号32位整数
 * @param dat        输入的4字节数组
 * @param big_endian 是否大端模式 (1=大端, 0=小端)
 * @return 转换后的int32_t
 */
static int32_t app_bytes_to_int32(const uint8_t dat[4], int big_endian)
{
    uint32_t val; // 先用无符号组装

    if (big_endian) {
        // 大端：高位在前
        val = ((uint32_t)dat[0] << 24) |
              ((uint32_t)dat[1] << 16) |
              ((uint32_t)dat[2] << 8)  |
              (uint32_t)dat[3];
    } else {
        // 小端：低位在前
        val = ((uint32_t)dat[3] << 24) |
              ((uint32_t)dat[2] << 16) |
              ((uint32_t)dat[1] << 8)  |
              (uint32_t)dat[0];
    }

    return (int32_t)val; // 直接强制转换为有符号
}


// 解析数据
static int app_parse_recv_data(uint8_t *data, uint32_t length, float *line_speed, float *angle_speed)
{
#ifdef DEBUG_PID
	unsigned char buf[32] = {0};

	strcpy((char *)buf, (char *)data);
	int temp_data = atoi((char *)&buf[2]);
	if (buf[0] == 'P') {
		*line_speed  = temp_data;
		printf("line_speed:%d\r\n");
	} else if (buf[0] == 'D') {
		*angle_speed = temp_data;
		printf("angle_speed:%d\r\n");
	} else {

	}
#else
	uint8_t temp_data[4] = {0};
	int i = 0;
	uint32_t temp_line_speed = 0;
	uint32_t temp_angle_speed = 0;
	uint8_t data_buf[16] = {0};
	uint16_t data_len = 0;
	uint8_t data_sun = 0;

	if (length != 16) {
		return -1;
	}
	memcpy(data_buf, data, length);

	if (data_buf[0] == 0x07 && data_buf[15] == 0x0A) {
		data_len = data_buf[1];
		for(i=2; i < data_len; i++) {
			data_sun += data_buf[i]; // 计算校验和
		}
		if(data_sun != data_buf[data_len + 1]) {
			return -1;
		}

		for (i=2; i < 2 + 4; i++) {
			temp_data[i] = data[i+2];
		}
		temp_line_speed = app_bytes_to_int32(temp_data, 0);
		*line_speed = temp_line_speed * 0.001;

		for (i=6; i < 6 + 4; i++) {
			temp_data[i] = data[i];
		}
		temp_angle_speed = app_bytes_to_int32(temp_data, 0);
		*angle_speed = temp_angle_speed * 0.001;

	}
#endif
	return 0;
}


void app(void)
{
	float speed_Ma = 0;
 	float speed_Mb = 0;
	float out_Speed = 0;
	float line_speed = 0;
	float angle_speed = 0;
	float target_line_speed = 0;
	float target_angle_speed = 0;
	float target_line_wheel_left = 0;
	float target_line_wheel_right = 0;
	// test_motor();
	while(1) {
#ifdef DEBUG_PID
		if (Rx2_Flag == 1) {
			Rx2_Flag = 0;
			printf("get it\r\n");
			app_get_buf_data(Rx2_Buf);
		}
#endif
		if (Rx2_Flag == 1) {
			Rx2_Flag = 0;
			app_parse_recv_data(Rx2_Buf, Rx2_Len, &target_line_speed, &target_angle_speed); // 解析接收到的数据

			app_diff_drive_calc_wheel_vel(target_line_speed, target_angle_speed, CAR_PARM_WHEEL_RTEAD,
			&target_line_wheel_left, &target_line_wheel_right);

			ctrl_balance_set_speed(target_line_speed);
		}

		get_euler_angle();
		ctrl_balance_update_speed(motor_1.foc.shaft_velocity, -motor_2.foc.shaft_velocity);
		ctrl_balance_update_angle(euler_angle.pitch, euler_angle.gyroy);
		ctrl_balance_vertical_task();
		ctrl_balance_speed_out(&out_Speed);
		speed_Ma = out_Speed + target_line_wheel_left;
		speed_Mb = -out_Speed - target_line_wheel_right;

		foc_loop_handle(&motor_1, speed_Ma);
		foc_loop_handle(&motor_2, speed_Mb);

		app_calc_robot_velocity(motor_1.foc.shaft_velocity, -motor_2.foc.shaft_velocity,
		CAR_PARM_WHEEL_RADI, CAR_PARM_WHEEL_RTEAD,
		&line_speed, &angle_speed);

		sprintf((char*)send_buf, "D:%d,%d,%d,%d,%d,%d,%d\r\n",
		(int)(1000 * line_speed),
		(int)(angle_speed),
		(int)(1000* app_wheel_angular_to_linear(motor_1.foc.shaft_angle, CAR_PARM_WHEEL_RADI)),
		(int)(1000* app_wheel_angular_to_linear(motor_2.foc.shaft_angle, CAR_PARM_WHEEL_RADI)),
		(int)euler_angle.pitch,
		(int)euler_angle.roll,
		(int)euler_angle.yaw
		);
		usart2_driver_Transmit(send_buf,sizeof(send_buf));
#ifdef DEBUG_PID
		sprintf((char*)send_buf, "D:%f,%f,%f,%f,%f,%f,%f,%f\r\n",
		(float)euler_angle.gyroy,
		euler_angle.pitch,
		motor_1.foc.shaft_velocity,
		motor_2.foc.shaft_velocity,
		motor_1.foc.shaft_angle,
		motor_2.foc.shaft_angle,
		motor_1.foc.electrical_angle,
		motor_2.foc.electrical_angle
		);
		usart2_driver_Transmit(send_buf,sizeof(send_buf));
#endif
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
