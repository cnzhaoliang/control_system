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

    const char *buffer;  // opemMV使用的串口缓冲
    uint16_t bufferSize; // opemMV使用的串口缓冲大小
    uint16_t *bufferSTA; // opemMV使用的串口缓冲状态

    IncrementalPIDControllerTypeDef yawController;   // 偏航角PID控制器
    IncrementalPIDControllerTypeDef pitchController; // 俯仰角PID控制器

    void (*RefreshThread)(struct GimbalType *self);                          // 云台的状态更新进程
    void (*Stop)(struct GimbalType *self);                                   // 关闭云台
    void (*Start)(struct GimbalType *self);                                  // 开启云台
    void (*SetTask)(struct GimbalType *self, uint8_t taskName);              // 设置云台任务
    void (*SetFreeAngle)(struct GimbalType *self, double yaw, double pitch); // 设置云台自由模式的姿态角

} GimbalTypeDef;

typedef struct GimbalInitType
{
    ServoInitTypeDef yawServoInit_Structure;
    ServoInitTypeDef pitchServoInit_Structure;
    const char *buffer;  // opemMV使用的串口缓冲
    uint16_t bufferSize; // opemMV使用的串口缓冲大小
    uint16_t *bufferSTA; // opemMV使用的串口缓冲状态
    IncrementalPIDControllerInitTypeDef yawControllerInit_Structure;
    IncrementalPIDControllerInitTypeDef pitchControllerInit_Structure;

} GimbalInitTypeDef;

// 云台初始化
void Gimbal_Init(GimbalTypeDef *Gimbal, GimbalInitTypeDef GimbalInit_Structure);

// 云台实例
extern GimbalTypeDef Gimbal;
// 云台实例初始化
void Laser_Gimbal_Init(void);

#define GIMBAL_NO_TASK 0
#define GIMBAL_FREE_TASK 1
#define GIMBAL_SLAVE_TASK 2

#endif
