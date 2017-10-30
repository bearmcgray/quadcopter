#ifndef __PID_H
#define __PID_H

typedef struct{
	float Kp;
	float Ki;
	float Kd;

	float max_lim;
	float min_lim;

	float output;
	float integral_tmp;
	float prev_error;

	float dt;

}tpid;

void PID_Init(tpid *pid);
void PID_SetCoefs(tpid *pid, float kp, float ki, float kd, float time);
void PID_SetLimits(tpid *pid, float max, float min);

float PID_Compute(tpid *pid, float setpoint, float input, float dinput );

float PID_GetOutput(tpid *pid);

void NVIC_ForTIM2_Configuration(void);
void TIM2_200Hz_Configuration(void);


#endif //__PID_H
