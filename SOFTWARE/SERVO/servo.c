#include "servo.h"

#ifdef DBG_Y_SERVO
ServoTypeDef yawServo; // 水平舵机
#endif

#ifdef DBG_P_SERVO
ServoTypeDef pitchServo; // 垂直舵机
#endif

/**********************************************************************************************************/

// 设置舵机的绝对角度
void SetAbsoluteAngle(ServoTypeDef *Servo, double angle)
{
    if (angle > Servo->private_angleRange)
    {
        angle = Servo->private_angleRange;
    }
    else if (angle < 0)
    {
        angle = 0;
    }
    uint32_t ccrValue = 0;
    uint32_t arrValue = __HAL_TIM_GET_AUTORELOAD(Servo->private_TIM_PWM);
    double duty = (Servo->private_signalMinWidth + (Servo->private_signalMaxWidth - Servo->private_signalMinWidth) * angle / Servo->private_angleRange) / Servo->private_signalPeriod;
    int ccrTemp = (double)(arrValue + 1) * duty; // 进行 CCR 寄存器值计算及范围校验
    if (ccrTemp > arrValue)
    {
        ccrValue = arrValue;
    }
    else if (ccrTemp < 0)
    {
        ccrValue = 0;
    }
    else
    {
        ccrValue = ccrTemp;
    }
    __HAL_TIM_SetCompare(Servo->private_TIM_PWM, Servo->private_PWM_Channel, ccrValue);
}

// 舵机状态更新进程
void p_Servo_Refresh_Thread(ServoTypeDef *Servo)
{
    if (Servo->initFlag == INITIALIZED)
    {
        if (Servo->runFlag == READY)
        {
            Servo->relativeAngleLimiter.signals.input = Servo->inputRelativeAngle;
            Servo->relativeAngleLimiter.RefreshThread(&Servo->relativeAngleLimiter);
            SetAbsoluteAngle(Servo, Servo->private_centerAngle + Servo->relativeAngleLimiter.signals.output);
        }
    }
}

// 关闭舵机
void p_Servo_stop(ServoTypeDef *Servo)
{
    if (Servo->initFlag == INITIALIZED)
    {
        Servo->runFlag = NOT_READY;
        SetAbsoluteAngle(Servo, Servo->private_centerAngle);
        Servo->relativeAngleLimiter.Stop(&Servo->relativeAngleLimiter);
        Servo->inputRelativeAngle = 0;
    }
}

// 开启舵机
void p_Servo_start(ServoTypeDef *Servo)
{
    if (Servo->initFlag == INITIALIZED)
    {
        SetAbsoluteAngle(Servo, Servo->private_centerAngle);
        Servo->relativeAngleLimiter.Start(&Servo->relativeAngleLimiter);
        Servo->inputRelativeAngle = 0;
        Servo->runFlag = READY;
    }
}

// 舵机初始化
void Servo_Init(ServoTypeDef *Servo, ServoInitTypeDef ServoInit_Structure)
{
    Servo->private_TIM_PWM = ServoInit_Structure.TIM_PWM;
    Servo->private_PWM_Channel = ServoInit_Structure.PWM_Channel;
    Servo->private_signalPeriod = ServoInit_Structure.signalPeriod;
    Servo->private_signalMinWidth = ServoInit_Structure.signalMinWidth;
    Servo->private_signalMaxWidth = ServoInit_Structure.signalMaxWidth;
    Servo->private_angleRange = ServoInit_Structure.angleRange;
    Servo->private_centerAngle = ServoInit_Structure.centerAngle;

    Servo->relativeAngleLimiter.maxValue = ServoInit_Structure.relativeAngleLimiterInit_Structure.maxValue;
    Servo->relativeAngleLimiter.minValue = ServoInit_Structure.relativeAngleLimiterInit_Structure.minValue;

    Limiter_Init(&Servo->relativeAngleLimiter, ServoInit_Structure.relativeAngleLimiterInit_Structure);

    Servo->RefreshThread = p_Servo_Refresh_Thread;
    Servo->Stop = p_Servo_stop;
    Servo->Start = p_Servo_start;

    SetAbsoluteAngle(Servo, Servo->private_centerAngle);
    HAL_TIM_PWM_Start(Servo->private_TIM_PWM, Servo->private_PWM_Channel);

    Servo->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

#ifdef DBG_Y_SERVO
// 水平云台实例初始化
void YawServo_Init(void)
{
    ServoInitTypeDef ServoInit_Structure = {0};

    ServoInit_Structure.TIM_PWM = &htim8;
    ServoInit_Structure.PWM_Channel = TIM_CHANNEL_1;
    ServoInit_Structure.signalPeriod = 20;
    ServoInit_Structure.signalMinWidth = 0.5;
    ServoInit_Structure.signalMaxWidth = 2.5;
    ServoInit_Structure.angleRange = 270;
    ServoInit_Structure.centerAngle = 135;

    ServoInit_Structure.relativeAngleLimiterInit_Structure.maxValue = 45;
    ServoInit_Structure.relativeAngleLimiterInit_Structure.minValue = -45;

    Servo_Init(&yawServo, ServoInit_Structure);
}
#endif
#ifdef DBG_P_SERVO
//  垂直云台实例初始化
void PitchServo_Init(void)
{
    ServoInitTypeDef ServoInit_Structure = {0};

    ServoInit_Structure.TIM_PWM = &htim8;
    ServoInit_Structure.PWM_Channel = TIM_CHANNEL_2;
    ServoInit_Structure.signalPeriod = 20;
    ServoInit_Structure.signalMinWidth = 0.5;
    ServoInit_Structure.signalMaxWidth = 2.5;
    ServoInit_Structure.angleRange = 270;
    ServoInit_Structure.centerAngle = 135;

    ServoInit_Structure.relativeAngleLimiterInit_Structure.maxValue = 45;
    ServoInit_Structure.relativeAngleLimiterInit_Structure.minValue = -45;

    Servo_Init(&pitchServo, ServoInit_Structure);
}
#endif
