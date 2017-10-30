#include "led.h"

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void LED_Init(tled *led) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// pin a0
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);

	// pin a8 - input key
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOA, GPIO_Pin_0);

	// pin b15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);

	// pin c14 c15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOC, GPIO_Pin_14 | GPIO_Pin_15);

}

void LED_Off(tled *led, int num) {

	led->state &= (~num) & LED_ALL
	;
	if (num & LED0) {
		GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	}
	if (num & LED1) {
		GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	}
	if (num & LED2) {
		GPIO_ResetBits(GPIOC, GPIO_Pin_14);
	}
	if (num & LED3) {
		GPIO_ResetBits(GPIOC, GPIO_Pin_15);
	}
}

void LED_On(tled *led, int num) {
	led->state |= num;
	if (num & LED0) {
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
	}
	if (num & LED1) {
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
	}
	if (num & LED2) {
		GPIO_SetBits(GPIOC, GPIO_Pin_14);
	}
	if (num & LED3) {
		GPIO_SetBits(GPIOC, GPIO_Pin_15);
	}
}

void LED_Toggle(tled *led, int num) {

	if (num & LED0) {
		if (led->state & LED0) {
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			led->state &= (~LED0) & LED_ALL;
		} else {
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			led->state |= LED0;
		}
	}
	if (num & LED1) {
		if (led->state & LED1) {
			GPIO_ResetBits(GPIOB, GPIO_Pin_15);
			led->state &= (~LED1) & LED_ALL;
		} else {
			GPIO_SetBits(GPIOB, GPIO_Pin_15);
			led->state |= LED1;
		}
	}
	if (num & LED2) {
		if (led->state & LED2) {
			GPIO_ResetBits(GPIOC, GPIO_Pin_14);
			led->state &= (~LED2) & LED_ALL;
		} else {
			GPIO_SetBits(GPIOC, GPIO_Pin_14);
			led->state |= LED2;
		}
	}
	if (num & LED3) {
		if (led->state & LED3) {
			GPIO_ResetBits(GPIOC, GPIO_Pin_15);
			led->state &= (~LED3) & LED_ALL;
		} else {
			GPIO_SetBits(GPIOC, GPIO_Pin_15);
			led->state |= LED3;
		}
	}
}
