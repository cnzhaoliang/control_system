#ifndef __SERVO_H__
#define __SERVO_H__

#include "control.h"
#include "tim.h"

typedef struct ServoType
{
    uint8_t initFlag; // ��ʼ����־
    uint8_t runFlag;  // ���б�־

    double inputRelativeAngle;

    TIM_HandleTypeDef *private_TIM_PWM;  // �������PWM�Ķ�ʱ��
    double private_PWM_Channel;          // �������PWM�Ķ�ʱ��ͨ��
    double private_signalPeriod;         // �ź�����
    double private_signalMinWidth;       // �ź���С���
    double private_signalMaxWidth;       // �ź������
    double private_angleRange;           // ����Ƕȷ�Χ
    double private_centerAngle;          // ������ĽǶ�
    LimiterTypeDef relativeAngleLimiter; // ��ԽǶ��޷���

    void (*RefreshThread)(struct ServoType *self); // �����״̬���½���
    void (*Stop)(struct ServoType *self);          // �رն��
    void (*Start)(struct ServoType *self);         // �������

} ServoTypeDef;

// ����ĳ�ʼ���ṹ��
typedef struct ServoInitType
{
    TIM_HandleTypeDef *TIM_PWM;                            // �������PWM�Ķ�ʱ��
    double PWM_Channel;                                    // �������PWM�Ķ�ʱ��ͨ��
    double signalPeriod;                                   // �ź�����
    double signalMinWidth;                                 // �ź���С���
    double signalMaxWidth;                                 // �ź������
    double angleRange;                                     // ����Ƕȷ�Χ
    double centerAngle;                                    // ������ĽǶ�
    LimiterInitTypeDef relativeAngleLimiterInit_Structure; // ��ԽǶ��޷����ĳ�ʼ���ṹ��

} ServoInitTypeDef;

#ifdef DBG_Y_SERVO
extern ServoTypeDef yawServo;
// ��ʼ��ˮƽ���ʵ��
void YawServo_Init(void);
#endif
#ifdef DBG_P_SERVO
extern ServoTypeDef pitchServo;
// ��ʼ����ֱ���ʵ��
void PitchServo_Init(void);
#endif

// ��ʼ�����
void Servo_Init(ServoTypeDef *Servo, ServoInitTypeDef ServoInit_Structure);

#endif
