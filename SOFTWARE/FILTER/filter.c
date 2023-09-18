#include "filter.h"

/**************************************************************************/

//

/**************************************************************************/

// 卡尔曼滤波器的状态更新进程
void p_KalmanFilterForMPU_RefreshThread(KalmanFilterForMPUTypeDef *KalmanFilterForMPU)
{
	if (KalmanFilterForMPU->initFlag == INITIALIZED)
	{
		if (KalmanFilterForMPU->runFlag == READY)
		{
			KalmanFilterForMPU->outputAngle += (KalmanFilterForMPU->inputGyro - KalmanFilterForMPU->private_Q_bias) * KalmanFilterForMPU->private_dt;				 // 先验估计
			KalmanFilterForMPU->private_Pdot[0] = KalmanFilterForMPU->private_Q_angle - KalmanFilterForMPU->private_PP[0][1] - KalmanFilterForMPU->private_PP[1][0]; // Pk-先验估计误差协方差的微分

			KalmanFilterForMPU->private_Pdot[1] = -KalmanFilterForMPU->private_PP[1][1];
			KalmanFilterForMPU->private_Pdot[2] = -KalmanFilterForMPU->private_PP[1][1];
			KalmanFilterForMPU->private_Pdot[3] = KalmanFilterForMPU->private_Q_gyro;
			KalmanFilterForMPU->private_PP[0][0] += KalmanFilterForMPU->private_Pdot[0] * KalmanFilterForMPU->private_dt; // Pk-先验估计误差协方差微分的积分
			KalmanFilterForMPU->private_PP[0][1] += KalmanFilterForMPU->private_Pdot[1] * KalmanFilterForMPU->private_dt; // =先验估计误差协方差
			KalmanFilterForMPU->private_PP[1][0] += KalmanFilterForMPU->private_Pdot[2] * KalmanFilterForMPU->private_dt;
			KalmanFilterForMPU->private_PP[1][1] += KalmanFilterForMPU->private_Pdot[3] * KalmanFilterForMPU->private_dt;

			KalmanFilterForMPU->private_Angle_err = KalmanFilterForMPU->inputAccel - KalmanFilterForMPU->outputAngle; // zk-先验估计

			KalmanFilterForMPU->private_PCt_0 = KalmanFilterForMPU->private_C_0 * KalmanFilterForMPU->private_PP[0][0];
			KalmanFilterForMPU->private_PCt_1 = KalmanFilterForMPU->private_C_0 * KalmanFilterForMPU->private_PP[1][0];

			KalmanFilterForMPU->private_E = KalmanFilterForMPU->private_R_angle + KalmanFilterForMPU->private_C_0 * KalmanFilterForMPU->private_PCt_0;

			KalmanFilterForMPU->private_K_0 = KalmanFilterForMPU->private_PCt_0 / KalmanFilterForMPU->private_E;
			KalmanFilterForMPU->private_K_1 = KalmanFilterForMPU->private_PCt_1 / KalmanFilterForMPU->private_E;

			KalmanFilterForMPU->private_t_0 = KalmanFilterForMPU->private_PCt_0;
			KalmanFilterForMPU->private_t_1 = KalmanFilterForMPU->private_C_0 * KalmanFilterForMPU->private_PP[0][1];

			KalmanFilterForMPU->private_PP[0][0] -= KalmanFilterForMPU->private_K_0 * KalmanFilterForMPU->private_t_0; // 后验估计误差协方差
			KalmanFilterForMPU->private_PP[0][1] -= KalmanFilterForMPU->private_K_0 * KalmanFilterForMPU->private_t_1;
			KalmanFilterForMPU->private_PP[1][0] -= KalmanFilterForMPU->private_K_1 * KalmanFilterForMPU->private_t_0;
			KalmanFilterForMPU->private_PP[1][1] -= KalmanFilterForMPU->private_K_1 * KalmanFilterForMPU->private_t_1;

			KalmanFilterForMPU->outputAngle += KalmanFilterForMPU->private_K_0 * KalmanFilterForMPU->private_Angle_err;	   // 后验估计
			KalmanFilterForMPU->private_Q_bias += KalmanFilterForMPU->private_K_1 * KalmanFilterForMPU->private_Angle_err; // 后验估计
			KalmanFilterForMPU->outputAngleDot = KalmanFilterForMPU->inputGyro - KalmanFilterForMPU->private_Q_bias;	   // 输出值(后验估计)的微分=角速度
		}
	}
}

// 关闭卡尔曼滤波器
void p_KalmanFilterForMPU_Stop(KalmanFilterForMPUTypeDef *KalmanFilterForMPU)
{
	KalmanFilterForMPU->runFlag = NOT_READY;

	KalmanFilterForMPU->private_Q_bias = 0;
	KalmanFilterForMPU->private_Angle_err = 0;
	KalmanFilterForMPU->private_PCt_0 = 0;
	KalmanFilterForMPU->private_PCt_1 = 0;
	KalmanFilterForMPU->private_E = 0;
	KalmanFilterForMPU->private_K_0 = 0;
	KalmanFilterForMPU->private_K_1 = 0;
	KalmanFilterForMPU->private_t_0 = 0;
	KalmanFilterForMPU->private_t_1 = 0;

	KalmanFilterForMPU->private_Q_angle = KalmanFilterForMPU->Init_Structure.Q_angle; // 过程噪声的协方差
	KalmanFilterForMPU->private_Q_gyro = KalmanFilterForMPU->Init_Structure.Q_gyro;	  // 过程噪声的协方差
	KalmanFilterForMPU->private_R_angle = KalmanFilterForMPU->Init_Structure.R_angle; // 测量噪声的协方差
	KalmanFilterForMPU->private_dt = KalmanFilterForMPU->Init_Structure.dt;			  // 采样周期
	KalmanFilterForMPU->private_C_0 = KalmanFilterForMPU->Init_Structure.C_0;
	memcpy(KalmanFilterForMPU->private_Pdot, KalmanFilterForMPU->Init_Structure.Pdot, sizeof(double[4]));
	memcpy(KalmanFilterForMPU->private_PP, KalmanFilterForMPU->Init_Structure.PP, sizeof(double[2][2]));

	KalmanFilterForMPU->inputAccel = 0;
	KalmanFilterForMPU->inputGyro = 0;
	KalmanFilterForMPU->outputAngle = 0;
	KalmanFilterForMPU->outputAngleDot = 0;
}

// 开启卡尔曼滤波器
void p_KalmanFilterForMPU_Start(KalmanFilterForMPUTypeDef *KalmanFilterForMPU)
{
	KalmanFilterForMPU->runFlag = READY;
}

// 初始化卡尔曼滤波器
void KalmanFilterForMPU_Init(KalmanFilterForMPUTypeDef *KalmanFilterForMPU, KalmanFilterForMPUInitTypeDef KalmanFilterForMPUInit_Structure)
{
	KalmanFilterForMPU->private_Q_angle = KalmanFilterForMPUInit_Structure.Q_angle; // 过程噪声的协方差
	KalmanFilterForMPU->private_Q_gyro = KalmanFilterForMPUInit_Structure.Q_gyro;	// 过程噪声的协方差
	KalmanFilterForMPU->private_R_angle = KalmanFilterForMPUInit_Structure.R_angle; // 测量噪声的协方差
	KalmanFilterForMPU->private_dt = KalmanFilterForMPUInit_Structure.dt;			// 采样周期
	KalmanFilterForMPU->private_C_0 = KalmanFilterForMPUInit_Structure.C_0;
	memcpy(KalmanFilterForMPU->private_Pdot, KalmanFilterForMPUInit_Structure.Pdot, sizeof(double[4]));
	memcpy(KalmanFilterForMPU->private_PP, KalmanFilterForMPUInit_Structure.PP, sizeof(double[2][2]));
	KalmanFilterForMPU->Init_Structure = KalmanFilterForMPUInit_Structure;

	KalmanFilterForMPU->RefreshThread = p_KalmanFilterForMPU_RefreshThread;
	KalmanFilterForMPU->Start = p_KalmanFilterForMPU_Start;
	KalmanFilterForMPU->Stop = p_KalmanFilterForMPU_Stop;

	KalmanFilterForMPU->initFlag = INITIALIZED;
}

/**************************************************************************/

// 一阶互补滤波器的状态更新进程
void p_ComplementaryFilterForMPU_RefreshThread(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU)
{
	ComplementaryFilterForMPU->outputAngle = (double)ComplementaryFilterForMPU->private_K1 * (double)ComplementaryFilterForMPU->inputAngle + (1 - (double)ComplementaryFilterForMPU->private_K1) * ((double)ComplementaryFilterForMPU->outputAngle + (double)ComplementaryFilterForMPU->inputGyro * CYCLE_S);
}

// 关闭一阶互补滤波器
void p_ComplementaryFilterForMPU_Stop(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU)
{
	ComplementaryFilterForMPU->runFlag = NOT_READY;

	ComplementaryFilterForMPU->private_K1 = ComplementaryFilterForMPU->Init_Structure.K1;

	ComplementaryFilterForMPU->inputAngle = 0;
	ComplementaryFilterForMPU->inputGyro = 0;
	ComplementaryFilterForMPU->outputAngle = 0;
}

// 开启一阶互补滤波器
void p_ComplementaryFilterForMPU_Start(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU)
{
	ComplementaryFilterForMPU->runFlag = READY;
}

// 初始化一阶互补滤波器
void ComplementaryFilterForMPU_Init(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU, ComplementaryFilterForMPUInitTypeDef ComplementaryFilterForMPUInit_Structure)
{
	ComplementaryFilterForMPU->private_K1 = ComplementaryFilterForMPUInit_Structure.K1;
	ComplementaryFilterForMPU->Init_Structure = ComplementaryFilterForMPUInit_Structure;

	ComplementaryFilterForMPU->RefreshThread = p_ComplementaryFilterForMPU_RefreshThread;
	ComplementaryFilterForMPU->Start = p_ComplementaryFilterForMPU_Start;
	ComplementaryFilterForMPU->Stop = p_ComplementaryFilterForMPU_Stop;

	ComplementaryFilterForMPU->initFlag = INITIALIZED;
}

/**************************************************************************/
