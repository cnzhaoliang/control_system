#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "servo.h"
#include "mpu6050.h"
#include "protocol.h"

typedef struct GimbalType
{
    uint8_t initFlag;
    uint8_t runFlag;
    uint8_t private_taskName;

    double inputFreeYaw;
    double inputFreePitch;

    uint8_t stopCMD;
    uint8_t startCMD;

    double debugErrorYaw;
    double debugErrorPitch;

    ServoTypeDef yawServo;
    ServoTypeDef pitchServo;

    const char *buffer;  // opemMVʹ�õĴ��ڻ���
    uint16_t bufferSize; // opemMVʹ�õĴ��ڻ����С
    uint16_t *bufferSTA; // opemMVʹ�õĴ��ڻ���״̬

    IncrementalPIDControllerTypeDef yawController;   // ƫ����PID������
    IncrementalPIDControllerTypeDef pitchController; // ������PID������

    void (*RefreshThread)(struct GimbalType *self);                          // ��̨��״̬���½���
    void (*Stop)(struct GimbalType *self);                                   // �ر���̨
    void (*Start)(struct GimbalType *self);                                  // ������̨
    void (*SetTask)(struct GimbalType *self, uint8_t taskName);              // ������̨����
    void (*SetFreeAngle)(struct GimbalType *self, double yaw, double pitch); // ������̨����ģʽ����̬��

} GimbalTypeDef;

typedef struct GimbalInitType
{
    ServoInitTypeDef yawServoInit_Structure;
    ServoInitTypeDef pitchServoInit_Structure;
    const char *buffer;  // opemMVʹ�õĴ��ڻ���
    uint16_t bufferSize; // opemMVʹ�õĴ��ڻ����С
    uint16_t *bufferSTA; // opemMVʹ�õĴ��ڻ���״̬
    IncrementalPIDControllerInitTypeDef yawControllerInit_Structure;
    IncrementalPIDControllerInitTypeDef pitchControllerInit_Structure;

} GimbalInitTypeDef;

// ��̨��ʼ��
void Gimbal_Init(GimbalTypeDef *Gimbal, GimbalInitTypeDef GimbalInit_Structure);

// ��̨ʵ��
extern GimbalTypeDef Gimbal;
// ��̨ʵ����ʼ��
void Laser_Gimbal_Init(void);

#define GIMBAL_NO_TASK 0
#define GIMBAL_FREE_TASK 1
#define GIMBAL_SLAVE_TASK 2

#endif
