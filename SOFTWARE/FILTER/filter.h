#ifndef __FILTER_H
#define __FILTER_H

#include "sys.h"
#include "delay.h"
#include "main.h"
#include "control.h"

// 卡尔曼滤波器的初始化结构体
typedef struct KalmanFilterForMPUInitType
{
  double Q_angle; // 过程噪声的协方差
  double Q_gyro;  // 过程噪声的协方差
  double R_angle; // 测量噪声的协方差
  double dt;      // 采样周期
  char C_0;
  double Pdot[4];
  double PP[2][2];

} KalmanFilterForMPUInitTypeDef;

// 定义卡尔曼滤波器类型
typedef struct KalmanFilterForMPUType
{
  uint8_t initFlag; // 初始化标志
  uint8_t runFlag;  // 运行标志

  double inputAccel;
  double inputGyro;
  double outputAngle;
  double outputAngleDot;

  double private_Q_angle; // 过程噪声的协方差
  double private_Q_gyro;  // 过程噪声的协方差
  double private_R_angle; // 测量噪声的协方差
  double private_dt;      // 采样周期
  char private_C_0;
  double private_Q_bias;
  double private_Angle_err;
  double private_PCt_0, private_PCt_1, private_E;
  double private_K_0, private_K_1, private_t_0, private_t_1;
  double private_Pdot[4];
  double private_PP[2][2];

  KalmanFilterForMPUInitTypeDef Init_Structure;

  void (*RefreshThread)(struct KalmanFilterForMPUType *self);
  void (*Stop)(struct KalmanFilterForMPUType *self);
  void (*Start)(struct KalmanFilterForMPUType *self);

} KalmanFilterForMPUTypeDef;

typedef struct ComplementaryFilterForMPUInitType
{
  double K1;

} ComplementaryFilterForMPUInitTypeDef;

typedef struct ComplementaryFilterForMPUType
{
  uint8_t initFlag; // 初始化标志
  uint8_t runFlag;  // 运行标志
  double inputAngle;
  double inputGyro;
  double outputAngle;
  double private_K1;

  ComplementaryFilterForMPUInitTypeDef Init_Structure;

  void (*RefreshThread)(struct ComplementaryFilterForMPUType *self);
  void (*Stop)(struct ComplementaryFilterForMPUType *self);
  void (*Start)(struct ComplementaryFilterForMPUType *self);

} ComplementaryFilterForMPUTypeDef;

// 初始化卡尔曼滤波器
void KalmanFilter_Init(KalmanFilterForMPUTypeDef *KalmanFilterForMPU);
// 初始化一阶互补滤波器
void ComplementaryFilterForMPU_Init(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU, ComplementaryFilterForMPUInitTypeDef ComplementaryFilterForMPUInit_Structure);

#endif
