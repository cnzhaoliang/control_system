#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "sys.h"
#include "delay.h"
#include "main.h"

#define CYCLE_S 0.005f
#define PI 3.1415926f

// 给类定义子类用类名，给类方法传入参数用类指针

// 定义时域信号类型
typedef struct SignalsType
{
	double input;  // 输入信号
	double output; // 输出信号

} SignalsTypeDef;

// 定义多路复用器类型
typedef struct MuxType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志
	uint8_t command;  // 选择信号

	double input_0; // 输入信号0
	double input_1; // 输入信号1
	double input_2; // 输入信号2
	double input_3; // 输入信号3
	double output;	// 输出信号

	void (*RefreshThread)(struct MuxType *Mux); // 多路复用器的状态更新进程
	void (*Stop)(struct MuxType *Mux);			// 关闭多路复用器
	void (*Start)(struct MuxType *Mux);			// 开启多路复用器

} MuxTypeDef;

// 多路复用器的初始化结构体
typedef struct MuxInitType
{
	uint8_t command; // 选择信号

} MuxInitTypeDef;

// 定义比例环节类型
typedef struct ProportionType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double Kp;				// 比例环节的比例系数

	void (*RefreshThread)(struct ProportionType *Proportion); // 比例环节的状态更新进程
	void (*Stop)(struct ProportionType *Proportion);		  // 关闭比例环节
	void (*Start)(struct ProportionType *Proportion);		  // 开启比例环节

} ProportionTypeDef;

// 比例环节初始化结构体
typedef struct ProportionInitType
{
	double Kp; // 比例环节的比例系数

} ProportionInitTypeDef;

// 定义积分器类型
typedef struct IntegratorType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double private_sum;

	void (*RefreshThread)(struct IntegratorType *Integrator); // 积分器的状态更新进程
	void (*Stop)(struct IntegratorType *Integrator);		  // 关闭积分器
	void (*Start)(struct IntegratorType *Integrator);		  // 开启积分器

} IntegratorTypeDef;

// 定义微分器类型
typedef struct DifferentiatorType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double private_valueBackwards;

	void (*RefreshThread)(struct DifferentiatorType *Differentiator); // 微分器的状态更新进程
	void (*Stop)(struct DifferentiatorType *Differentiator);		  // 关闭微分器
	void (*Start)(struct DifferentiatorType *Differentiator);		  // 开启微分器

} DifferentiatorTypeDef;

// 定义增量式PID控制器类型
typedef struct IncrementalPIDControllerType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double Kp;				// 控制器的P参数
	double Ki;				// 控制器的I参数
	double Kd;				// 控制器的D参数
	double private_controlBackwards1;
	double private_errorBackwards1;
	double private_errorBackwards2;

	void (*RefreshThread)(struct IncrementalPIDControllerType *PIDController); // 增量式PID控制器的状态更新进程
	void (*Stop)(struct IncrementalPIDControllerType *PIDController);		   // 关闭增量式PID控制器
	void (*Start)(struct IncrementalPIDControllerType *PIDController);		   // 开启增量式PID控制器

} IncrementalPIDControllerTypeDef;

// 增量式PID控制器的初始化结构体
typedef struct IncrementalPIDControllerInitType
{
	double Kp;
	double Ki;
	double Kd;

} IncrementalPIDControllerInitTypeDef;

// 定义补偿器类型
typedef struct CompensatorType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double poszoneWidth;	// 正向补偿值宽度
	double negzoneWidth;	// 负向补偿宽度

	void (*RefreshThread)(struct CompensatorType *Compensator); // 补偿器的状态更新进程
	void (*Stop)(struct CompensatorType *Compensator);			// 关闭补偿器
	void (*Start)(struct CompensatorType *Compensator);			// 开启补偿器

} CompensatorTypeDef;

// 补偿器的初始化结构体
typedef struct CompensatorInitType
{
	double poszoneWidth;
	double negzoneWidth;

} CompensatorInitTypeDef;

// 定义限幅器类型
typedef struct LimiterType
{
	uint8_t initFlag; // 初始化标志
	uint8_t runFlag;  // 运行标志

	SignalsTypeDef signals; // 输入输出信号集合
	double maxValue;		// 输出的最大值
	double minValue;		// 输出的最小值

	void (*RefreshThread)(struct LimiterType *Limiter); // 限幅器的状态更新进程
	void (*Stop)(struct LimiterType *Limiter);			// 关闭限幅器
	void (*Start)(struct LimiterType *Limiter);			// 开启限幅器

} LimiterTypeDef;

// 限幅器的初始化结构体
typedef struct LimiterInitType
{
	double maxValue;
	double minValue;

} LimiterInitTypeDef;

// 初始化多路复用器
void Mux_Init(MuxTypeDef *Mux, MuxInitTypeDef MuxInit_Structure);
// 初始化比例环节
void Proportion_Init(ProportionTypeDef *Proportion, ProportionInitTypeDef ProportionInit_Structure);
// 初始化积分器
void Integrator_Init(IntegratorTypeDef *Integrator);
// 初始化微分器
void Differentiator_Init(DifferentiatorTypeDef *Differentiator);
// 初始化增量式PID控制器
void IncrementalPIDController_Init(IncrementalPIDControllerTypeDef *PIDController, IncrementalPIDControllerInitTypeDef IncrementalPIDControllerInit_Structure);
// 初始化限幅器
void Limiter_Init(LimiterTypeDef *Limiter, LimiterInitTypeDef LimiterInit_Structure);
// 初始化补偿器
void Compensator_Init(CompensatorTypeDef *Compensator, CompensatorInitTypeDef CompensatorInit_Structure);

#define UNINITIALIZED 0x00
#define INITIALIZED 0x01

#define NOT_READY 0x00
#define READY 0x01

#endif
