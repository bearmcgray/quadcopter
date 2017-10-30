#include "analog_drv.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"

void setzero(void* dat, int sz) {
	char *d = dat;
	for (int i = 0; i < sz; ++i)
		d[i] = 0;
}

float middle(tmiddler *dat, float par) {
	float sum = 0;
	dat->ary[dat->sptr++] = par;
	if (dat->sptr == dat->steps)
		dat->sptr = 0;
	for (int i = 0; i < dat->steps; ++i)
		sum += dat->ary[i];
	dat->midval = sum / dat->steps;
	return dat->midval;
}

void middle_init(tmiddler *dat, int par) {
	setzero(dat,sizeof(tmiddler));
	if( par > MAX_STEPS )par=MAX_STEPS;
	dat->steps = par;
}

void Analog_Init() {

	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3,ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC3 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 regular channel7 configuration *************************************/
	ADC_RegularChannelConfig(ADC3, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

}

int Analog_GetCurrent() {
	ADC_SoftwareStartConv(ADC3);
	while( !ADC_GetFlagStatus(ADC3,ADC_FLAG_EOC) ){
		__asm("nop");
	}
	return ADC_GetConversionValue(ADC3);
}
