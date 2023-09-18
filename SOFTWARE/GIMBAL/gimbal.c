#include "gimbal.h"

GimbalTypeDef Gimbal;

/**********************************************************************************************************/

void GetMsgFromBuffer(const char *buffer, uint16_t bufferSize, uint16_t *bufferSTA, double *errorYawAdress, double *errorPitchAdress, uint8_t *stopCMDAdress, uint8_t *startCMDAdress)
{
    char jsonStrBuffer[bufferSize];
    if ((bufferSize != 0) && (*bufferSTA) == NOT_EMPTY)
    {
        memcpy(jsonStrBuffer, buffer, bufferSize);
        cJSON *root = cJSON_Parse((const char *)jsonStrBuffer);
        if (root == NULL)
        {
            printf("Failed to parse JSON: %s\n", cJSON_GetErrorPtr());
        }
        else
        {
            cJSON *errorYaw = cJSON_GetObjectItemCaseSensitive(root, "X");
            cJSON *errorPitch = cJSON_GetObjectItemCaseSensitive(root, "Y");
            cJSON *stopCMD = cJSON_GetObjectItemCaseSensitive(root, "stopCMD");
            cJSON *startCMD = cJSON_GetObjectItemCaseSensitive(root, "startCMD");
            if (cJSON_IsNumber(errorYaw))
            {
                *errorYawAdress = errorYaw->valuedouble;
            }
            else
            {
                //*errorYawAdress = 0;
            }
            if (cJSON_IsNumber(errorPitch))
            {
                *errorPitchAdress = errorPitch->valuedouble;
            }
            else
            {
                //*errorPitchAdress = 0;
            }
            if (cJSON_IsNumber(stopCMD))
            {
                *stopCMDAdress = stopCMD->valuedouble;
            }
            else
            {
                //*stopCMDAdress = 0;
            }
            if (cJSON_IsNumber(startCMD))
            {
                *startCMDAdress = startCMD->valuedouble;
            }
            else
            {
                //*startCMDAdress = 0;
            }
            if (!cJSON_IsNumber(errorYaw) && !cJSON_IsNumber(errorPitch) && !cJSON_IsNumber(stopCMD))
            {
                printf("JSON format error\n");
            }
        }
        cJSON_Delete(root); // 释放资源
        *bufferSTA = EMPTY;
    }
}

// 云台状态更新进程
void p_Gimbal_Refresh_Thread(GimbalTypeDef *Gimbal)
{
    if (Gimbal->initFlag == INITIALIZED)
    {
        if (Gimbal->runFlag == READY)
        {
            if (Gimbal->private_taskName == GIMBAL_FREE_TASK)
            {
                Gimbal->yawServo.inputRelativeAngle = Gimbal->inputFreeYaw;
                Gimbal->pitchServo.inputRelativeAngle = Gimbal->inputFreePitch;
                Gimbal->yawServo.RefreshThread(&Gimbal->yawServo);
                Gimbal->pitchServo.RefreshThread(&Gimbal->pitchServo);
            }
            else if (Gimbal->private_taskName == GIMBAL_SLAVE_TASK)
            {
                GetMsgFromBuffer(Gimbal->buffer, Gimbal->bufferSize, Gimbal->bufferSTA, &Gimbal->debugErrorYaw, &Gimbal->debugErrorPitch, &Gimbal->stopCMD, &Gimbal->startCMD);
                Gimbal->yawController.signals.input = Gimbal->debugErrorYaw;
                Gimbal->pitchController.signals.input = -Gimbal->debugErrorPitch;
                Gimbal->yawController.RefreshThread(&Gimbal->yawController);
                Gimbal->pitchController.RefreshThread(&Gimbal->pitchController);
                Gimbal->yawServo.inputRelativeAngle += Gimbal->yawController.signals.output;
                Gimbal->pitchServo.inputRelativeAngle += Gimbal->pitchController.signals.output;
                Gimbal->yawServo.RefreshThread(&Gimbal->yawServo);
                Gimbal->pitchServo.RefreshThread(&Gimbal->pitchServo);
            }
        }
    }
}

// 关闭云台
void p_Gimbal_Stop(GimbalTypeDef *Gimbal)
{
    if (Gimbal->initFlag == INITIALIZED)
    {
        Gimbal->runFlag = NOT_READY;
        Gimbal->yawServo.Stop(&Gimbal->yawServo);
        Gimbal->pitchServo.Stop(&Gimbal->pitchServo);
        Gimbal->yawController.Stop(&Gimbal->yawController);
        Gimbal->pitchController.Stop(&Gimbal->pitchController);
        Gimbal->inputFreeYaw = 0;
        Gimbal->inputFreePitch = 0;
        Gimbal->debugErrorYaw = 0;
        Gimbal->debugErrorPitch = 0;
        *(Gimbal->bufferSTA) = EMPTY;
    }
}

// 开启云台
void p_Gimbal_Start(GimbalTypeDef *Gimbal)
{
    if (Gimbal->initFlag == INITIALIZED)
    {
        Gimbal->yawServo.Start(&Gimbal->yawServo);
        Gimbal->pitchServo.Start(&Gimbal->pitchServo);
        Gimbal->yawController.Start(&Gimbal->yawController);
        Gimbal->pitchController.Start(&Gimbal->pitchController);
        Gimbal->inputFreeYaw = 0;
        Gimbal->inputFreePitch = 0;
        Gimbal->debugErrorYaw = 0;
        Gimbal->debugErrorPitch = 0;
        *(Gimbal->bufferSTA) = EMPTY;
        Gimbal->runFlag = READY;
    }
}

// 设置云台任务
void p_Gimbal_SetTask(GimbalTypeDef *Gimbal, uint8_t taskName)
{
    if (Gimbal->initFlag == INITIALIZED)
    {
        if (Gimbal->private_taskName != taskName)
        {
            if (Gimbal->private_taskName == GIMBAL_NO_TASK)
            {
                Gimbal->private_taskName = taskName;
            }
            else
            {
                Gimbal->Stop(Gimbal);
                Gimbal->private_taskName = taskName;
                delay_ms(1000);
                Gimbal->Start(Gimbal);
            }
        }
    }
}

// 设置云台自由模式的姿态角
void p_Gimbal_SetFreeAngle(GimbalTypeDef *Gimbal, double yaw, double pitch)
{
    if (Gimbal->initFlag == INITIALIZED)
    {
        if (Gimbal->runFlag == READY)
        {
            if (Gimbal->private_taskName == GIMBAL_FREE_TASK)
            {
                Gimbal->inputFreeYaw = yaw;
                Gimbal->inputFreePitch = pitch;
            }
        }
    }
}

// 云台初始化
void Gimbal_Init(GimbalTypeDef *Gimbal, GimbalInitTypeDef GimbalInit_Structure)
{
    Gimbal->buffer = GimbalInit_Structure.buffer;
    Gimbal->bufferSize = GimbalInit_Structure.bufferSize;
    Gimbal->bufferSTA = GimbalInit_Structure.bufferSTA;
    Servo_Init(&Gimbal->yawServo, GimbalInit_Structure.yawServoInit_Structure);
    Servo_Init(&Gimbal->pitchServo, GimbalInit_Structure.pitchServoInit_Structure);
    IncrementalPIDController_Init(&Gimbal->yawController, GimbalInit_Structure.yawControllerInit_Structure);
    IncrementalPIDController_Init(&Gimbal->pitchController, GimbalInit_Structure.pitchControllerInit_Structure);

    Gimbal->RefreshThread = p_Gimbal_Refresh_Thread;
    Gimbal->Stop = p_Gimbal_Stop;
    Gimbal->Start = p_Gimbal_Start;
    Gimbal->SetTask = p_Gimbal_SetTask;
    Gimbal->SetFreeAngle = p_Gimbal_SetFreeAngle;

    Gimbal->initFlag = INITIALIZED;
}

// 云台实例初始化
void Laser_Gimbal_Init(void)
{
    GimbalInitTypeDef GimbalInit_Structure = {0};

    GimbalInit_Structure.buffer = (const char *)UART_Frame.COM2_BUF;
    GimbalInit_Structure.bufferSize = DATA_MAX_LEN;
    GimbalInit_Structure.bufferSTA = &UART_Frame.COM2_STA;

    GimbalInit_Structure.yawControllerInit_Structure.Kp = 0.005;
    GimbalInit_Structure.yawControllerInit_Structure.Ki = 0;
    GimbalInit_Structure.yawControllerInit_Structure.Kd = 0;

    GimbalInit_Structure.pitchControllerInit_Structure.Kp = 0.005;
    GimbalInit_Structure.pitchControllerInit_Structure.Ki = 0;
    GimbalInit_Structure.pitchControllerInit_Structure.Kd = 0;

    GimbalInit_Structure.yawServoInit_Structure.TIM_PWM = &htim8;
    GimbalInit_Structure.yawServoInit_Structure.PWM_Channel = TIM_CHANNEL_1;
    GimbalInit_Structure.yawServoInit_Structure.signalPeriod = 20;
    GimbalInit_Structure.yawServoInit_Structure.signalMinWidth = 0.5;
    GimbalInit_Structure.yawServoInit_Structure.signalMaxWidth = 2.5;
    GimbalInit_Structure.yawServoInit_Structure.angleRange = 270;
    GimbalInit_Structure.yawServoInit_Structure.centerAngle = 135;
    GimbalInit_Structure.yawServoInit_Structure.relativeAngleLimiterInit_Structure.maxValue = 45;
    GimbalInit_Structure.yawServoInit_Structure.relativeAngleLimiterInit_Structure.minValue = -45;

    GimbalInit_Structure.pitchServoInit_Structure.TIM_PWM = &htim8;
    GimbalInit_Structure.pitchServoInit_Structure.PWM_Channel = TIM_CHANNEL_2;
    GimbalInit_Structure.pitchServoInit_Structure.signalPeriod = 20;
    GimbalInit_Structure.pitchServoInit_Structure.signalMinWidth = 0.5;
    GimbalInit_Structure.pitchServoInit_Structure.signalMaxWidth = 2.5;
    GimbalInit_Structure.pitchServoInit_Structure.angleRange = 180;
    GimbalInit_Structure.pitchServoInit_Structure.centerAngle = 72;
    GimbalInit_Structure.pitchServoInit_Structure.relativeAngleLimiterInit_Structure.maxValue = 45;
    GimbalInit_Structure.pitchServoInit_Structure.relativeAngleLimiterInit_Structure.minValue = -45;

    Gimbal_Init(&Gimbal, GimbalInit_Structure);
}
