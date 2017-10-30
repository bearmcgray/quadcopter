#include "uart_log.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "misc.h"
#include <math.h>


void usart_init(USART_TypeDef* USARTx){
	/* USARTx configured as follow:
        - BaudRate = 115200 baud
        - Word Length = 8 Bits
        - One Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled
	 */
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

	assert_param( ((USARTx==USART1)||(USARTx==USART3)) );

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;

	if(USARTx==USART1){
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
		/* Enable UART clock */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

		/* Connect PXx to USARTx_Tx*/
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		/* Connect PXx to USARTx_Rx*/
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

		/* Configure USART Tx as alternate function  */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		/* Configure USART Rx as alternate function  */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	// we want to configure the USART1 interrupts

	}else{//USART3
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
		/* Enable UART clock */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

		/* Connect PXx to USARTx_Tx*/
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
		/* Connect PXx to USARTx_Rx*/
		GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

		/* Configure USART Tx as alternate function  */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		/* Configure USART Rx as alternate function  */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_Init(GPIOC, &GPIO_InitStructure);

		NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	// we want to configure the USART1 interrupts
	}

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USARTx, &USART_InitStructure);

	/* Here the USART1 receive interrupt is enabled
	 * and the interrupt controller is configured
	 * to jump to the USART1_IRQHandler() function
	 * if the USART1 receive interrupt occurs
	 */
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART1 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	// this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	// the USART1 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART */
	USART_Cmd(USARTx, ENABLE);
}

void wr_str(USART_TypeDef* USARTx, void* data){
	char* str = (char*)data;
	while(*str){
		USART_SendData(USARTx,*str);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		{}
		str++;
	}
}

void wr_buf(USART_TypeDef* USARTx, void* data, int size){
	char* buf = (char*)data;
	for (int i = 0; i<size; ++i){
		USART_SendData(USARTx,buf[i]);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		{}
	}
}

void wr_group(USART_TypeDef* USARTx, int data0, int data1, int data2){
	wr_int(USARTx,data0);
	USART_SendData(USARTx,',');
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
		__asm("nop");
	}
	wr_int(USARTx,data1);
	USART_SendData(USARTx,',');
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
		__asm("nop");
	}
	wr_int(USARTx,data2);
	USART_SendData(USARTx,'\r');
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
		__asm("nop");
	}
}

void wr_long_group(USART_TypeDef* USARTx, int data0, int data1, int data2, int data3, int data4, int data5){
	int data[] = {data0,data1,data2,data3,data4,data5};
	for(int i = 0; i<6; ++i){
		wr_int(USARTx,data[i]);
		if(i==5){
			USART_SendData(USARTx,'\r');
		}else{
			USART_SendData(USARTx,',');
		}
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
			__asm("nop");
		}
	}

}

void wr_long_group9(USART_TypeDef* USARTx, int data0, int data1, int data2, int data3, int data4, int data5, int data6, int data7, int data8){
	int n = 9;
	int data[] = {data0,data1,data2,data3,data4,data5,data6,data7,data8};

	for(int i = 0; i<n; ++i){
		wr_int(USARTx,data[i]);
		if(i==(n-1)){
			USART_SendData(USARTx,'\r');
		}else{
			USART_SendData(USARTx,',');
		}
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET){
			__asm("nop");
		}
	}

}


void wr_int(USART_TypeDef* USARTx, int data){
	char str[12];
	int len=1;
	int pos;
	int tmp=data;
	while(tmp/10){
		tmp/=10;
		len++;
	}
	if(tmp < 0){
		len++;
		str[0]='-';
	}
	//char *str = new char[len+1];
	pos=len;
	str[pos]='\0';
	//str[pos]='\r';
	//str[pos+1]='\0';
	tmp = data;
	while( ( (data>=0)&&pos )||( (data<0)&&pos>1 ) ){
		str[--pos]=abs(tmp)%10+0x30;
		tmp/=10;
	}
	pos = 0;
	while(str[pos]){
		USART_SendData(USARTx,str[pos]);
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET)
		{}
		pos++;
	}
}
