#ifndef __FILTER_H
#define __FILTER_H

#include "sys.h"
#include "delay.h"
#include "main.h"
#include "control.h"

// �������˲����ĳ�ʼ���ṹ��
typedef struct KalmanFilterForMPUInitType
{
  double Q_angle; // ����������Э����
  double Q_gyro;  // ����������Э����
  double R_angle; // ����������Э����
  double dt;      // ��������
  char C_0;
  double Pdot[4];
  double PP[2][2];

} KalmanFilterForMPUInitTypeDef;

// ���忨�����˲�������
typedef struct KalmanFilterForMPUType
{
  uint8_t initFlag; // ��ʼ����־
  uint8_t runFlag;  // ���б�־

  double inputAccel;
  double inputGyro;
  double outputAngle;
  double outputAngleDot;

  double private_Q_angle; // ����������Э����
  double private_Q_gyro;  // ����������Э����
  double private_R_angle; // ����������Э����
  double private_dt;      // ��������
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
  uint8_t initFlag; // ��ʼ����־
  uint8_t runFlag;  // ���б�־
  double inputAngle;
  double inputGyro;
  double outputAngle;
  double private_K1;

  ComplementaryFilterForMPUInitTypeDef Init_Structure;

  void (*RefreshThread)(struct ComplementaryFilterForMPUType *self);
  void (*Stop)(struct ComplementaryFilterForMPUType *self);
  void (*Start)(struct ComplementaryFilterForMPUType *self);

} ComplementaryFilterForMPUTypeDef;

// ��ʼ���������˲���
void KalmanFilter_Init(KalmanFilterForMPUTypeDef *KalmanFilterForMPU);
// ��ʼ��һ�׻����˲���
void ComplementaryFilterForMPU_Init(ComplementaryFilterForMPUTypeDef *ComplementaryFilterForMPU, ComplementaryFilterForMPUInitTypeDef ComplementaryFilterForMPUInit_Structure);

#endif
