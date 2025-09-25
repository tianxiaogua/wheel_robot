#include "Kalman.h"

/*
Q值为过程噪声，越小系统越容易收敛，我们对模型预测的值信任度越高；但是太小则容易发散，如果Q为零，那么我们只相信预测值；
Q值越大我们对于预测的信任度就越低，而对测量值的信任度就变高；如果Q值无穷大，那么我们只信任测量值；

R值为测量噪声，太小太大都不一定合适。R太大，卡尔曼滤波响应会变慢，因为它对新测量的值的信任度降低；越小系统收敛越快，
但过小则容易出现震荡；R值的改变主要是影响卡尔曼的收敛速度。

测试时可以先将Q从小往大调整，将R从大往小调整；先固定一个值去调整另外一个值，看收敛速度与波形输出。

系统中还有一个关键值P，它是误差协方差初始值，表示我们对当前预测状态的信任度，它越小说明我们越相信当前预测状态；
它的值决定了初始收敛速度，一般开始设一个较小的值以便于获取较快的收敛速度。随着卡尔曼滤波的迭代，
P的值会不断的改变，当系统进入稳态之后P值会收敛成一个最小的估计方差矩阵，这个时候的卡尔曼增益也是最优的，
所以这个值只是影响初始收敛速度。
————————————————
版权声明：本文为CSDN博主「三木今天学习了嘛」的原创文章，遵循CC 4.0 BY-SA版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/weixin_45751396/article/details/119595886
*/

Kalman kfp1;
void Kalman1_Init()
{
	kfp1.Last_P = 1;			
	kfp1.Now_P = 0;		
	kfp1.out = 0;			
	kfp1.Kg = 0;		
	kfp1.Q = 0.001f;
	kfp1.R = 0.02f;
}

Kalman kfp2;
void Kalman2_Init()
{
	kfp2.Last_P = 1;			
	kfp2.Now_P = 0;		
	kfp2.out = 0;			
	kfp2.Kg = 0;		
	kfp2.Q = 0.001f;
	kfp2.R = 0.02f;
}

Kalman kfp;
void Kalman_Init()
{
	kfp.Last_P = 1;			
	kfp.Now_P = 0;		
	kfp.out = 0;			
	kfp.Kg = 0;		
	kfp.Q = 0.001f; // 越小系统越容易收敛，我们对模型预测的值信任度越高；但是太小则容易发散，如果Q为零，那么我们只相信预测值；
	kfp.R = 0.02f;
}

/**
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter(Kalman *kfp,float input)		
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);//因为这一次的预测值就是上一次的输出值
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 威下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}



// #include "kalman.h"

void Kanman_Init(KALMAN_STRUCT * kalman)
{
	int i;
	
	//输出
	(*kalman).Angel = 0.0;	//最优估计的角度	是最终角度结果
	(*kalman).Gyro_x = 0.0;	//最优估计角速度
	
	//固定参量
	(*kalman).Q_Angle = 0.001;		//{0.001,0.001,0.001};	//陀螺仪噪声协方差	0.001是经验值
	(*kalman).Q_Gyro = 0.003;		//{0.003,0.003,0.003};	//陀螺仪漂移噪声协方差	是mpu6050的经验值
	(*kalman).R_Angle = 0.5;		//{0.5,0.5,0.5};	//是加速度计噪声的协方差	
	
	(*kalman).C_0 = 1;		//{1,1,1};	//H矩阵的一个观测参数 是常数
	
	//中间量
	(*kalman).Q_Bias = 0;		//{0,0,0};		//陀螺仪飘移预估值
	(*kalman).Angle_err = 0;	//{0,0,0};		//计算中间值 Angle 观测值-预估值
	
	(*kalman).PCt_0 = 0;			//{0,0,0},	//计算中间值
	(*kalman).PCt_1 = 0;			//{0,0,0},
	(*kalman).E     = 0;			//{0,0,0};
	(*kalman).t_0   = 0;			//{0,0,0},	//t:计算中间变量
	(*kalman).t_1   = 0;			//{0,0,0},
	
	(*kalman).K_0 = 0;			//{0,0,0},	//K:卡尔曼增益
	(*kalman).K_1 = 0;			//{0,0,0},
	
	for(i = 0;i < 4;i++)	//{0,0,0,0}	//计算P矩阵的中间矩阵
	{
		(*kalman).Pdot[i] = 0;
	}
	
	(*kalman).PP[0][0] = 1;
	(*kalman).PP[0][1] = 0;
	(*kalman).PP[1][0] = 0;
	(*kalman).PP[1][1] = 1;
}


void Kanman_Filter(KALMAN_STRUCT * kalman,float Gyro,float Accel,uint32_t dt)	//Gyro陀螺仪的测量值  |  Accel加速度计的角度计  |  dt的时间考虑用小数 或 更小的分度表示
{
	float dt_f;
	
	//把dt这个单位是ms的u32型变量里的值转换为float型的以秒为单位的值
	dt_f = (float)dt;
	dt_f = dt_f / 1000;
	
	//x轴指向前，y轴指向左的坐标系   要算俯仰角
	//那么输入的应该是y轴的角速度（Gyro）和y轴的倾角加速度计估计值
	//坐标系情况大概是这样
	
	
	//角度测量模型方程 角度估计值=上一次最有角度+（角速度-上一次的最优零飘）*dt_f
	//就漂移来说，认为每次都是相同的Q_bias=Q_bias
	//估计角度
	(*kalman).Angel += (Gyro - (*kalman).Q_Bias) * dt_f;
	
	//计算估计模型的方差
	(*kalman).Pdot[0] = (*kalman).Q_Angle - (*kalman).PP[0][1] - (*kalman).PP[1][0];
	(*kalman).Pdot[1] = -(*kalman).PP[1][1];
	(*kalman).Pdot[2] = -(*kalman).PP[1][1];
	(*kalman).Pdot[3] = (*kalman).Q_Gyro;
	
	(*kalman).PP[0][0] += (*kalman).Pdot[0] * dt_f;
	(*kalman).PP[0][1] += (*kalman).Pdot[1] * dt_f;
	(*kalman).PP[1][0] += (*kalman).Pdot[2] * dt_f;
	(*kalman).PP[1][1] += (*kalman).Pdot[3] * dt_f;
	
	//计算卡尔曼增益
	(*kalman).PCt_0 = (*kalman).C_0 * (*kalman).PP[0][0];	//矩阵乘法的中间变量
	(*kalman).PCt_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//C_0=1
	(*kalman).E = (*kalman).R_Angle + (*kalman).C_0 * (*kalman).PCt_0;	//分母
	(*kalman).K_0 = (*kalman).PCt_0 / (*kalman).E;	//卡尔曼增益，两个，一个是Angle的，一个是Q_bias的
	(*kalman).K_1 = (*kalman).PCt_1 / (*kalman).E;
	
	//计算最优角度、最优零飘
	(*kalman).Angle_err = Accel - (*kalman).Angel;
	(*kalman).Angel += (*kalman).K_0 * (*kalman).Angle_err;	//计算最优的角度
	(*kalman).Q_Bias += (*kalman).K_1 * (*kalman).Angle_err;	//计算最优的零飘
	
	(*kalman).Gyro_x = Gyro -(*kalman).Q_Bias;	//计算得最优角速度
	
	//更新估计模型的方差
	(*kalman).t_0 = (*kalman).PCt_0;	//矩阵计算中间变量，相当于a
	(*kalman).t_1 = (*kalman).C_0 * (*kalman).PP[0][1];	//矩阵计算中间变量，相当于b
	
	(*kalman).PP[0][0] -= (*kalman).K_0 * (*kalman).t_0;
	(*kalman).PP[0][1] -= (*kalman).K_0 * (*kalman).t_1;
	(*kalman).PP[1][0] -= (*kalman).K_1 * (*kalman).t_0;
	(*kalman).PP[1][1] -= (*kalman).K_1 * (*kalman).t_1;
}
