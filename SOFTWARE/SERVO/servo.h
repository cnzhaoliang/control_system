#ifndef __SERVO_H__
#define __SERVO_H__

#include "control.h"
#include "tim.h"

typedef struct ServoType
{
    uint8_t initFlag; // 初始化标志
    uint8_t runFlag;  // 运行标志

    double inputRelativeAngle;

    TIM_HandleTypeDef *private_TIM_PWM;  // 驱动舵机PWM的定时器
    double private_PWM_Channel;          // 驱动舵机PWM的定时器通道
    double private_signalPeriod;         // 信号周期
    double private_signalMinWidth;       // 信号最小宽度
    double private_signalMaxWidth;       // 信号最大宽度
    double private_angleRange;           // 舵机角度范围
    double private_centerAngle;          // 舵机中心角度
    LimiterTypeDef relativeAngleLimiter; // 相对角度限幅器

    void (*RefreshThread)(struct ServoType *self); // 舵机的状态更新进程
    void (*Stop)(struct ServoType *self);          // 关闭舵机
    void (*Start)(struct ServoType *self);         // 开启舵机

} ServoTypeDef;

// 舵机的初始化结构体
typedef struct ServoInitType
{
    TIM_HandleTypeDef *TIM_PWM;                            // 驱动舵机PWM的定时器
    double PWM_Channel;                                    // 驱动舵机PWM的定时器通道
    double signalPeriod;                                   // 信号周期
    double signalMinWidth;                                 // 信号最小宽度
    double signalMaxWidth;                                 // 信号最大宽度
    double angleRange;                                     // 舵机角度范围
    double centerAngle;                                    // 舵机中心角度
    LimiterInitTypeDef relativeAngleLimiterInit_Structure; // 相对角度限幅器的初始化结构体

} ServoInitTypeDef;

#ifdef DBG_Y_SERVO
extern ServoTypeDef yawServo;
// 初始化水平舵机实例
void YawServo_Init(void);
#endif
#ifdef DBG_P_SERVO
extern ServoTypeDef pitchServo;
// 初始化垂直舵机实例
void PitchServo_Init(void);
#endif

// 初始化舵机
void Servo_Init(ServoTypeDef *Servo, ServoInitTypeDef ServoInit_Structure);

#endif
