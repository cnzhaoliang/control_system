#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "sys.h"
#include "delay.h"
#include "main.h"

#define CYCLE_S 0.005f
#define PI 3.1415926f

// ���ඨ�����������������෽�������������ָ��

// ����ʱ���ź�����
typedef struct SignalsType
{
	double input;  // �����ź�
	double output; // ����ź�

} SignalsTypeDef;

// �����·����������
typedef struct MuxType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־
	uint8_t command;  // ѡ���ź�

	double input_0; // �����ź�0
	double input_1; // �����ź�1
	double input_2; // �����ź�2
	double input_3; // �����ź�3
	double output;	// ����ź�

	void (*RefreshThread)(struct MuxType *Mux); // ��·��������״̬���½���
	void (*Stop)(struct MuxType *Mux);			// �رն�·������
	void (*Start)(struct MuxType *Mux);			// ������·������

} MuxTypeDef;

// ��·�������ĳ�ʼ���ṹ��
typedef struct MuxInitType
{
	uint8_t command; // ѡ���ź�

} MuxInitTypeDef;

// ���������������
typedef struct ProportionType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double Kp;				// �������ڵı���ϵ��

	void (*RefreshThread)(struct ProportionType *Proportion); // �������ڵ�״̬���½���
	void (*Stop)(struct ProportionType *Proportion);		  // �رձ�������
	void (*Start)(struct ProportionType *Proportion);		  // ������������

} ProportionTypeDef;

// �������ڳ�ʼ���ṹ��
typedef struct ProportionInitType
{
	double Kp; // �������ڵı���ϵ��

} ProportionInitTypeDef;

// �������������
typedef struct IntegratorType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double private_sum;

	void (*RefreshThread)(struct IntegratorType *Integrator); // ��������״̬���½���
	void (*Stop)(struct IntegratorType *Integrator);		  // �رջ�����
	void (*Start)(struct IntegratorType *Integrator);		  // ����������

} IntegratorTypeDef;

// ����΢��������
typedef struct DifferentiatorType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double private_valueBackwards;

	void (*RefreshThread)(struct DifferentiatorType *Differentiator); // ΢������״̬���½���
	void (*Stop)(struct DifferentiatorType *Differentiator);		  // �ر�΢����
	void (*Start)(struct DifferentiatorType *Differentiator);		  // ����΢����

} DifferentiatorTypeDef;

// ��������ʽPID����������
typedef struct IncrementalPIDControllerType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double Kp;				// ��������P����
	double Ki;				// ��������I����
	double Kd;				// ��������D����
	double private_controlBackwards1;
	double private_errorBackwards1;
	double private_errorBackwards2;

	void (*RefreshThread)(struct IncrementalPIDControllerType *PIDController); // ����ʽPID��������״̬���½���
	void (*Stop)(struct IncrementalPIDControllerType *PIDController);		   // �ر�����ʽPID������
	void (*Start)(struct IncrementalPIDControllerType *PIDController);		   // ��������ʽPID������

} IncrementalPIDControllerTypeDef;

// ����ʽPID�������ĳ�ʼ���ṹ��
typedef struct IncrementalPIDControllerInitType
{
	double Kp;
	double Ki;
	double Kd;

} IncrementalPIDControllerInitTypeDef;

// ���岹��������
typedef struct CompensatorType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double poszoneWidth;	// ���򲹳�ֵ���
	double negzoneWidth;	// ���򲹳����

	void (*RefreshThread)(struct CompensatorType *Compensator); // ��������״̬���½���
	void (*Stop)(struct CompensatorType *Compensator);			// �رղ�����
	void (*Start)(struct CompensatorType *Compensator);			// ����������

} CompensatorTypeDef;

// �������ĳ�ʼ���ṹ��
typedef struct CompensatorInitType
{
	double poszoneWidth;
	double negzoneWidth;

} CompensatorInitTypeDef;

// �����޷�������
typedef struct LimiterType
{
	uint8_t initFlag; // ��ʼ����־
	uint8_t runFlag;  // ���б�־

	SignalsTypeDef signals; // ��������źż���
	double maxValue;		// ��������ֵ
	double minValue;		// �������Сֵ

	void (*RefreshThread)(struct LimiterType *Limiter); // �޷�����״̬���½���
	void (*Stop)(struct LimiterType *Limiter);			// �ر��޷���
	void (*Start)(struct LimiterType *Limiter);			// �����޷���

} LimiterTypeDef;

// �޷����ĳ�ʼ���ṹ��
typedef struct LimiterInitType
{
	double maxValue;
	double minValue;

} LimiterInitTypeDef;

// ��ʼ����·������
void Mux_Init(MuxTypeDef *Mux, MuxInitTypeDef MuxInit_Structure);
// ��ʼ����������
void Proportion_Init(ProportionTypeDef *Proportion, ProportionInitTypeDef ProportionInit_Structure);
// ��ʼ��������
void Integrator_Init(IntegratorTypeDef *Integrator);
// ��ʼ��΢����
void Differentiator_Init(DifferentiatorTypeDef *Differentiator);
// ��ʼ������ʽPID������
void IncrementalPIDController_Init(IncrementalPIDControllerTypeDef *PIDController, IncrementalPIDControllerInitTypeDef IncrementalPIDControllerInit_Structure);
// ��ʼ���޷���
void Limiter_Init(LimiterTypeDef *Limiter, LimiterInitTypeDef LimiterInit_Structure);
// ��ʼ��������
void Compensator_Init(CompensatorTypeDef *Compensator, CompensatorInitTypeDef CompensatorInit_Structure);

#define UNINITIALIZED 0x00
#define INITIALIZED 0x01

#define NOT_READY 0x00
#define READY 0x01

#endif
