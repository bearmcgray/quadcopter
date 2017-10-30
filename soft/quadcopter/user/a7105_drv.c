/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.

 */
#include "stm32f4xx.h"
#include "a7105_drv.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "spi_bus.h"

static const u8 A7105_regs[51] = {
//       0     1     2     3     4    5    6     7     8     9     a     b     c     d     e     f
		-1, 0x42, 0x00, 0x14, 0x00,  -1,  -1,  0x00, 0x00, 0x00, 0x00, 0x19, 0x01, 0x05, 0x00, 0x50,
	  0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62,0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
	  0x13, 0xc3, 0x00,   -1, 0x00, 0x00, 0x3b, 0x00, 0x1f, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
	  0x01,	0x0f, -1 };

char ch_tb_9[16] = { 0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28,
		0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50 };

#define A7105_CS_DOWN GPIO_ResetBits(GPIOA, GPIO_Pin_12);
#define A7105_CS_UP GPIO_SetBits(GPIOA, GPIO_Pin_12);

int A7105_WaitWTF() {
	int delay = 100000;
	//A7105_WriteReg(0x0b, 0x01);

	int prev = 0;
	int cur = 0;
	int to = 0;
	while (!((prev) && (!cur)) && (to < delay)) {
		prev = cur;
		cur = GPIO_ReadInputData(GPIOD) & GPIO_Pin_2;
		++to;
	}

	//A7105_WriteReg(0x0b, 0x19);
	return to >= delay ? 0 : 1;
}


void A7105_WriteReg(u8 address, u8 data) {
	A7105_CS_DOWN

	SPI_ReadWriteByte(address);
	SPI_ReadWriteByte(data);

	A7105_CS_UP
}

u8 A7105_ReadReg(u8 address) {
	u8 data;

	A7105_CS_DOWN

	SPI_ReadWriteByte(READ_BIT_FLAG | address);
	data = SPI_ReadWriteByte(DUMMY_BYTE);

	A7105_CS_UP

	return data;
}

void A7105_WriteData(u8 *dpbuffer, u8 len, u8 channel) {
	int i;

	A7105_Strobe(A7105_RST_WRPTR);

	A7105_CS_DOWN

	SPI_ReadWriteByte(0x05);
	for (i = 0; i < len; i++)
		SPI_ReadWriteByte(dpbuffer[i]);
	A7105_CS_UP

	A7105_WriteReg(0x0F, channel);

	A7105_Strobe(A7105_TX);

	int prev = 0;
	int cur = 0;
	while (!((prev) && (!cur)) ) {
		prev = cur;
		cur = GPIO_ReadInputData(GPIOD) & GPIO_Pin_2;
	}
}

void A7105_ReadData(u8 *dpbuffer, u8 len) {
	A7105_CS_DOWN

	SPI_ReadWriteByte(0x45);

	for (int i = 0; i < len; ++i) {
		dpbuffer[i] = SPI_ReadWriteByte(DUMMY_BYTE);
	}

	A7105_CS_UP
}

void A7105_Reset() {
	A7105_WriteReg(0x00, 0x00);
}

void A7105_WriteID(u32 id) {
	A7105_CS_DOWN
	SPI_ReadWriteByte(0x06);

	SPI_ReadWriteByte(id >> 24);

	SPI_ReadWriteByte(id >> 16);

	SPI_ReadWriteByte(id >> 8);

	SPI_ReadWriteByte(id >> 0);

	A7105_CS_UP
}

u32 A7105_ReadID() {
	u32 data = 0;
	A7105_CS_DOWN

	SPI_ReadWriteByte(READ_BIT_FLAG | 0x06);
	data |= SPI_ReadWriteByte(DUMMY_BYTE) << 24;
	data |= SPI_ReadWriteByte(DUMMY_BYTE) << 16;
	data |= SPI_ReadWriteByte(DUMMY_BYTE) << 8;
	data |= SPI_ReadWriteByte(DUMMY_BYTE);

	A7105_CS_UP

	return data;
}

void A7105_Strobe(enum A7105_State state) {
	A7105_CS_DOWN
	SPI_ReadWriteByte(state);
	A7105_CS_UP
}

void A7105_Initialize() {
	A7105_CS_UP

	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	A7105_WriteReg(0x00, 0x00);

	for (int i = 0; i < 0x3fffff; ++i) {
		__asm("nop");
	}

	for (int i = 0; i < 51; ++i) {
		if (A7105_regs[i] != 0xff)
			A7105_WriteReg(i, A7105_regs[i]);
	}

	//A7105_WriteReg(0x0b, 0x19);

	////////////
	//IF Filter Bank Calibration
	A7105_WriteReg(0x02, 0x01);
	while (A7105_ReadReg(0x02) & 0x01)
		__asm("nop");

	//test calibration ok ?

	//VCO Current Calibration
	A7105_WriteReg(0x24, 0x13);

	//VCO Bank Calibration
	A7105_WriteReg(0x26, 0x3B);

	//VCO Bank Calibrate channel 0x00?
	A7105_WriteReg(0x0F, 0x00); //	;Set Channel 0
	A7105_WriteReg(0x02, 0x02); //	;VCO Calibration
	while (A7105_ReadReg(0x02) & 0x02)
		__asm("nop");

	//test calibration ok ?

	//VCO Bank Calibrate channel 0xa0?
	A7105_WriteReg(0x0F, 0xA0); //Set Channel 0xa0
	A7105_WriteReg(0x02, 0x02); //VCO Calibration
	while (A7105_ReadReg(0x02) & 0x02)
		__asm("nop");
	//test calibration ok ?

	//Reset VCO Band calibration
	A7105_WriteReg(0x25, 0x08);


    //A7105_SetPower(Model.tx_power);

    A7105_Strobe(A7105_STANDBY);
}

void A7105_SwitchChanel(u32 p){
	static u32 index = 0;
	if(p == -1){
		index = 0;
	}else{
		index +=p;
		if(index>15){
			index -=16;
		}
	}
	A7105_Strobe(A7105_STANDBY); //      ;A0 -> standby mode
	A7105_Strobe(A7105_RST_RDPTR); //      ;F0 -> reset RX FIFO
	A7105_WriteReg(0x0F, ch_tb_9[index] - 8); //	;set channel
	A7105_Strobe(A7105_RX); //      ;C0 -> get into RX mode
}
