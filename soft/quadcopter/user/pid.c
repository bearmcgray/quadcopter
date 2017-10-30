#include "pid.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"


void PID_Init(tpid *pid){
	char *d = (void*)pid;
	for(int i= 0; i<sizeof(tpid);++i)
		d[i]=0;
}

void PID_SetCoefs(tpid *pid, float kp, float ki, float kd, float time){
	pid->Kp = kp;
	pid->Ki = ki;
	pid->Kd = kd;
	pid->dt = time;
}

void PID_SetLimits(tpid *pid, float max, float min) {

	if (min > max)
		return;
	pid->max_lim = max;
	pid->min_lim = min;

	if (pid->output > pid->max_lim){
		pid->output = pid->max_lim;
	}else{
		if (pid->output < pid->min_lim){
			pid->output = pid->min_lim;
		}
	}

	if (pid->integral_tmp > pid->max_lim){
		pid->integral_tmp = pid->max_lim;
	}else{
		if (pid->integral_tmp < pid->min_lim){
			pid->integral_tmp = pid->min_lim;
		}
	}
}

float PID_Compute(tpid *pid, float setpoint, float input, float dinput){
	float error = setpoint - input;

	pid->integral_tmp += pid->Ki * error * pid->dt;

	if(pid->integral_tmp > pid->max_lim){
		pid->integral_tmp = pid->max_lim;
	}else{
		if(pid->integral_tmp < pid->min_lim){
			pid->integral_tmp = pid->min_lim;
		}
	}

	pid->output = pid->Kp*error
				+ pid->integral_tmp
				- pid->Kd*dinput;

	if(pid->output > pid->max_lim){
		pid->output = pid->max_lim;
	}else{
		if(pid->output < pid->min_lim){
			pid->output = pid->min_lim;
		}
	}

	return pid->output;
}

float PID_GetOutput(tpid *pid){
	return pid->output;
}


void TIM2_200Hz_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;

	TIM_DeInit( TIM2);                              //¸´Î»TIM2¶¨Ê±Æ÷
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);   //Ê¹ÄÜ¶¨Ê±Æ÷2
	//APB2=72M, TIM1_CLK=72/72=1MHz
	//TIM_Period£/TIM1_ARR£©=1000£¬¼ÆÊýÆ÷ÏòÉÏ¼ÆÊýµ½1000ºó²úÉú¸üÐÂÊÂ¼þ£¬¼ÆÊýÖµ¹éÁã
	//ÏòÉÏ¼ÆÊýÄ£Ê½
	TIM_BaseInitStructure.TIM_Period = 5000*4;//5000;//2500;
	TIM_BaseInitStructure.TIM_Prescaler = 167;
	TIM_BaseInitStructure.TIM_ClockDivision = 0;
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure);


	TIM_ClearFlag(TIM2, TIM_FLAG_Update);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
}

void NVIC_ForTIM2_Configuration(void)
{
  	NVIC_InitTypeDef NVIC_InitStructure;

	#ifdef  VECT_TAB_RAM
	  /* Set the Vector Table base location at 0x20000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
	#else  /* VECT_TAB_FLASH  */
	  /* Set the Vector Table base location at 0x08000000 */
	  NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	#endif

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}
