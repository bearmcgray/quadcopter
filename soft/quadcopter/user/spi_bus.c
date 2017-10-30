#include "spi_bus.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

 unsigned char SPI_ReadWriteByte(unsigned char TxData) {
 unsigned char RxData = 0;

 while (SPI_I2S_GetFlagStatus(SPI_BUS, SPI_I2S_FLAG_TXE) == RESET) {
 __asm("nop");
 }
 SPI_I2S_SendData(SPI_BUS, TxData);

 while (SPI_I2S_GetFlagStatus(SPI_BUS, SPI_I2S_FLAG_RXNE) == RESET) {
 __asm("nop");
 }

 RxData = SPI_I2S_ReceiveData(SPI_BUS);

 return (unsigned char) RxData;
 }

 void SPI_Bus_Init() {

 GPIO_InitTypeDef GPIO_InitStructure;
 SPI_InitTypeDef SPI_InitStructure;

 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
 RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

 RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 |GPIO_Pin_5 | GPIO_Pin_4;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_Init(GPIOB, &GPIO_InitStructure);

 GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI3);
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI3);
 GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI3);



 //Configure PA12 and PC13 pin: CS pin
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
 GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
 GPIO_Init(GPIOA, &GPIO_InitStructure);

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
 GPIO_Init(GPIOC, &GPIO_InitStructure);

 SPI_I2S_DeInit(SPI_BUS);
 // SPI1 configuration
 SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
 SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
 SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
 SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
 SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
 SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
 SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
 SPI_InitStructure.SPI_CRCPolynomial = 7;
 SPI_Init(SPI_BUS, &SPI_InitStructure);

 GPIO_SetBits(GPIOA,GPIO_Pin_12);
 GPIO_SetBits(GPIOC,GPIO_Pin_13);

 // Enable SPI1
 SPI_Cmd(SPI_BUS, ENABLE);
 }

/*
unsigned char SPI_ReadWriteByte(unsigned char TxData) {
	unsigned char RxData = 0;

	for (int i = 0; i < 8; ++i) {
		//GPIO_ResetBits(GPIOB,GPIO_Pin_3);//clk
		if(TxData&0x80){
			GPIO_SetBits(GPIOB,GPIO_Pin_5);//mosi
		}else{
			GPIO_ResetBits(GPIOB,GPIO_Pin_5);//mosi
		}
		GPIO_SetBits(GPIOB,GPIO_Pin_3);//clk
		TxData<<=1;
		RxData<<=1;
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)){

			RxData|=1;
		}
		GPIO_ResetBits(GPIOB,GPIO_Pin_3);//clk
	}

	return (unsigned char) RxData;
}

void SPI_Bus_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//Configure PA12 and PC13 pin: CS pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, GPIO_Pin_12);
	GPIO_SetBits(GPIOC, GPIO_Pin_13);
	GPIO_ResetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);

}
*/
