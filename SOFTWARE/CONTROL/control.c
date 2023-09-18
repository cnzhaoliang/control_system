#include "control.h"

/**********************************************************************************************************/

// 多路复用器状态更新进程
void p_Mux_Refresh_Thread(MuxTypeDef *Mux)
{
	if (Mux->initFlag == INITIALIZED)
	{
		if (Mux->runFlag == READY)
		{
			switch (Mux->command)
			{
			case 0:
				Mux->output = Mux->input_0;
				break;
			case 1:
				Mux->output = Mux->input_1;
				break;
			case 2:
				Mux->output = Mux->input_2;
				break;
			case 3:
				Mux->output = Mux->input_3;
				break;
			default:
				break;
			}
		}
	}
}

// 关闭多路复用器
void p_Mux_Stop(MuxTypeDef *Mux)
{
	if (Mux->initFlag == INITIALIZED)
	{
		Mux->runFlag = NOT_READY;
		Mux->input_0 = 0;
		Mux->input_1 = 0;
		Mux->input_2 = 0;
		Mux->input_3 = 0;
		Mux->output = 0;
	}
}

// 开启多路复用器
void p_Mux_Start(MuxTypeDef *Mux)
{
	if (Mux->initFlag == INITIALIZED)
	{
		Mux->input_0 = 0;
		Mux->input_1 = 0;
		Mux->input_2 = 0;
		Mux->input_3 = 0;
		Mux->output = 0;
		Mux->runFlag = READY;
	}
}

// 初始化多路复用器
void Mux_Init(MuxTypeDef *Mux, MuxInitTypeDef MuxInit_Structure)
{
	Mux->command = MuxInit_Structure.command;

	Mux->RefreshThread = p_Mux_Refresh_Thread;
	Mux->Stop = p_Mux_Stop;
	Mux->Start = p_Mux_Start;

	Mux->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 比例环节状态更新进程
void p_Proportion_Refresh_Thread(ProportionTypeDef *Proportion)
{
	if (Proportion->initFlag == INITIALIZED)
	{
		if (Proportion->runFlag == READY)
		{
			Proportion->signals.output = Proportion->signals.input * Proportion->Kp;
		}
	}
}

// 关闭比例环节
void p_Proportion_Stop(ProportionTypeDef *Proportion)
{
	if (Proportion->initFlag == INITIALIZED)
	{
		Proportion->runFlag = NOT_READY;
		Proportion->signals.input = 0;
		Proportion->signals.output = 0;
	}
}

// 开启比例环节
void p_Proportion_Start(ProportionTypeDef *Proportion)
{
	if (Proportion->initFlag == INITIALIZED)
	{
		Proportion->signals.input = 0;
		Proportion->signals.output = 0;
		Proportion->runFlag = READY;
	}
}

// 初始化比例环节
void Proportion_Init(ProportionTypeDef *Proportion, ProportionInitTypeDef ProportionInit_Structure)
{
	Proportion->Kp = ProportionInit_Structure.Kp;

	Proportion->RefreshThread = p_Proportion_Refresh_Thread;
	Proportion->Stop = p_Proportion_Stop;
	Proportion->Start = p_Proportion_Start;

	Proportion->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 积分器状态更新进程
void p_Integrator_Refresh_Thread(IntegratorTypeDef *Integrator)
{
	if (Integrator->initFlag == INITIALIZED)
	{
		if (Integrator->runFlag == READY)
		{
			Integrator->private_sum += Integrator->signals.input * CYCLE_S;
			Integrator->signals.output = Integrator->private_sum;
		}
	}
}

// 关闭积分器
void p_Integrator_Stop(IntegratorTypeDef *Integrator)
{
	if (Integrator->initFlag == INITIALIZED)
	{
		Integrator->runFlag = NOT_READY;
		Integrator->private_sum = 0;
		Integrator->signals.input = 0;
		Integrator->signals.output = 0;
	}
}

// 开启积分器
void p_Integrator_Start(IntegratorTypeDef *Integrator)
{
	if (Integrator->initFlag == INITIALIZED)
	{
		Integrator->private_sum = 0;
		Integrator->signals.input = 0;
		Integrator->signals.output = 0;
		Integrator->runFlag = READY;
	}
}

// 初始化积分器
void Integrator_Init(IntegratorTypeDef *Integrator)
{
	Integrator->RefreshThread = p_Integrator_Refresh_Thread;
	Integrator->Stop = p_Integrator_Stop;
	Integrator->Start = p_Integrator_Start;

	Integrator->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 微分环节状态更新进程
void p_Differentiator_Refresh_Thread(DifferentiatorTypeDef *Differentiator)
{
	if (Differentiator->initFlag == INITIALIZED)
	{
		if (Differentiator->runFlag == READY)
		{
			Differentiator->signals.output = (Differentiator->signals.input - Differentiator->private_valueBackwards) / CYCLE_S;
			Differentiator->private_valueBackwards = Differentiator->signals.input;
		}
	}
}

// 关闭微分器
void p_Differentiator_Stop(DifferentiatorTypeDef *Differentiator)
{
	if (Differentiator->initFlag == INITIALIZED)
	{
		Differentiator->runFlag = NOT_READY;
		Differentiator->private_valueBackwards = 0;
		Differentiator->signals.input = 0;
		Differentiator->signals.output = 0;
	}
}

// 开启微分器
void p_Differentiator_Start(DifferentiatorTypeDef *Differentiator)
{
	if (Differentiator->initFlag == INITIALIZED)
	{
		Differentiator->private_valueBackwards = 0;
		Differentiator->signals.input = 0;
		Differentiator->signals.output = 0;
		Differentiator->runFlag = READY;
	}
}

// 初始化微分器
void Differentiator_Init(DifferentiatorTypeDef *Differentiator)
{
	Differentiator->RefreshThread = p_Differentiator_Refresh_Thread;
	Differentiator->Stop = p_Differentiator_Stop;
	Differentiator->Start = p_Differentiator_Start;

	Differentiator->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 增量式PID控制器状态更新进程
void p_Incremental_PID_Controller_Refresh_Thread(IncrementalPIDControllerTypeDef *PIDController)
{
	if (PIDController->initFlag == INITIALIZED)
	{
		if (PIDController->runFlag == READY)
		{
			// 加载PID参数
			double Kp = PIDController->Kp;
			double Ki = PIDController->Ki;
			double Kd = PIDController->Kd;
			// 加载控制器状态
			double u_k_1 = PIDController->private_controlBackwards1;
			double e_k_1 = PIDController->private_errorBackwards1;
			double e_k_2 = PIDController->private_errorBackwards2;
			// 加载输入信号
			double e_k = PIDController->signals.input;
			// 计算输出信号
			double u_k = u_k_1 + Kp * (e_k - e_k_1) + Ki * CYCLE_S * e_k + Kd / CYCLE_S * (e_k - 2 * e_k_1 + e_k_2);
			// 更新输出信号
			PIDController->signals.output = u_k;
			// 更新控制器状态
			PIDController->private_controlBackwards1 = u_k;
			PIDController->private_errorBackwards1 = e_k;
			PIDController->private_errorBackwards2 = e_k_1;
		}
	}
}

// 关闭增量式PID控制器
void p_Incremental_PID_Controller_Stop(IncrementalPIDControllerTypeDef *PIDController)
{
	if (PIDController->initFlag == INITIALIZED)
	{
		PIDController->runFlag = NOT_READY;
		PIDController->private_controlBackwards1 = 0;
		PIDController->private_errorBackwards1 = 0;
		PIDController->private_errorBackwards2 = 0;
		PIDController->signals.input = 0;
		PIDController->signals.output = 0;
	}
}

// 开启增量式PID控制器
void p_Incremental_PID_Controller_Start(IncrementalPIDControllerTypeDef *PIDController)
{
	if (PIDController->initFlag == INITIALIZED)
	{
		PIDController->private_controlBackwards1 = 0;
		PIDController->private_errorBackwards1 = 0;
		PIDController->private_errorBackwards2 = 0;
		PIDController->signals.input = 0;
		PIDController->signals.output = 0;
		PIDController->runFlag = READY;
	}
}

// 初始化增量式PID控制器
void IncrementalPIDController_Init(IncrementalPIDControllerTypeDef *PIDController, IncrementalPIDControllerInitTypeDef IncrementalPIDControllerInit_Structure)
{
	PIDController->Kp = IncrementalPIDControllerInit_Structure.Kp;
	PIDController->Ki = IncrementalPIDControllerInit_Structure.Ki;
	PIDController->Kd = IncrementalPIDControllerInit_Structure.Kd;

	PIDController->RefreshThread = p_Incremental_PID_Controller_Refresh_Thread;
	PIDController->Stop = p_Incremental_PID_Controller_Stop;
	PIDController->Start = p_Incremental_PID_Controller_Start;

	PIDController->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 限幅器状态更新进程
void p_Limiter_Refresh_Thread(LimiterTypeDef *Limiter)
{
	if (Limiter->initFlag == INITIALIZED)
	{
		if (Limiter->runFlag == READY)
		{
			if (Limiter->signals.input > Limiter->maxValue)
			{
				Limiter->signals.output = Limiter->maxValue;
			}
			else if (Limiter->signals.input < Limiter->minValue)
			{
				Limiter->signals.output = Limiter->minValue;
			}
			else
			{
				Limiter->signals.output = Limiter->signals.input;
			}
		}
	}
}

// 关闭限幅器
void p_Limiter_Stop(LimiterTypeDef *Limiter)
{
	if (Limiter->initFlag == INITIALIZED)
	{
		Limiter->runFlag = NOT_READY;
		Limiter->signals.input = 0;
		Limiter->signals.output = 0;
	}
}

// 开启限幅器
void p_Limiter_Start(LimiterTypeDef *Limiter)
{
	if (Limiter->initFlag == INITIALIZED)
	{
		Limiter->signals.input = 0;
		Limiter->signals.output = 0;
		Limiter->runFlag = READY;
	}
}

// 初始化限幅器
void Limiter_Init(LimiterTypeDef *Limiter, LimiterInitTypeDef LimiterInit_Structure)
{
	Limiter->maxValue = LimiterInit_Structure.maxValue;
	Limiter->minValue = LimiterInit_Structure.minValue;

	Limiter->RefreshThread = p_Limiter_Refresh_Thread;
	Limiter->Stop = p_Limiter_Stop;
	Limiter->Start = p_Limiter_Start;

	Limiter->initFlag = INITIALIZED;
}

/**********************************************************************************************************/

// 补偿器状态更新进程
void p_Compensator_Refresh_Thread(CompensatorTypeDef *Compensator)
{
	if (Compensator->initFlag == INITIALIZED)
	{
		if (Compensator->runFlag == READY)
		{
			if (Compensator->signals.input > 0)
			{
				Compensator->signals.output = Compensator->signals.input + Compensator->poszoneWidth;
			}
			else if (Compensator->signals.input < 0)
			{
				Compensator->signals.output = Compensator->signals.input - Compensator->negzoneWidth;
			}
			else
			{
				Compensator->signals.output = 0;
			}
		}
	}
}

// 关闭补偿器
void p_Compensator_Stop(CompensatorTypeDef *Compensator)
{
	if (Compensator->initFlag == INITIALIZED)
	{
		Compensator->runFlag = NOT_READY;
		Compensator->signals.input = 0;
		Compensator->signals.output = 0;
	}
}

// 开启补偿器
void p_Compensator_Start(CompensatorTypeDef *Compensator)
{
	if (Compensator->initFlag == INITIALIZED)
	{
		Compensator->signals.input = 0;
		Compensator->signals.output = 0;
		Compensator->runFlag = READY;
	}
}

// 初始化补偿器
void Compensator_Init(CompensatorTypeDef *Compensator, CompensatorInitTypeDef CompensatorInit_Structure)
{
	Compensator->poszoneWidth = CompensatorInit_Structure.poszoneWidth;
	Compensator->negzoneWidth = CompensatorInit_Structure.negzoneWidth;

	Compensator->RefreshThread = p_Compensator_Refresh_Thread;
	Compensator->Stop = p_Compensator_Stop;
	Compensator->Start = p_Compensator_Start;

	Compensator->initFlag = INITIALIZED;
}

/**********************************************************************************************************/
