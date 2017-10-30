#include "stm32f4xx.h"
#include "spi_bus.h"
#include "at45DB161.h"
#include "stm32f4xx_gpio.h"

#define FLASH_CS_DOWN GPIO_ResetBits(GPIOC, GPIO_Pin_13);
#define FLASH_CS_UP GPIO_SetBits(GPIOC, GPIO_Pin_13);

void AT45DB_Init() {
	/*GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Configure PB5 pin: AT45DB_CS pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// SPI1 configuration
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	// Enable SPI1
	SPI_Cmd(SPI1, ENABLE);*/
//	AT45DB_CS = 1;
}

uint8_t AT45DB_ReadSR(void) {
	uint8_t byte = 0;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_RD_STATUS_REG);
	byte = SPI_ReadWriteByte(AT45DB_DUMMY_BYTE);
	FLASH_CS_UP;
	return byte;
}

void AT45DB_Repair512(void) {

	if (!(AT45DB_ReadSR() & AT45DB_PGSZ_BIT)) {

		FLASH_CS_DOWN;
		SPI_ReadWriteByte(AT45DB_PGSZ512_BYTE1);
		SPI_ReadWriteByte(AT45DB_PGSZ512_BYTE2);
		SPI_ReadWriteByte(AT45DB_PGSZ512_BYTE3);
		SPI_ReadWriteByte(AT45DB_PGSZ512_BYTE4);
		FLASH_CS_UP;

		AT45DB_Wait_Busy();
	}
}

uint32_t AT45DB_ReadID(void) {
	uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

	/* Select the FLASH: Chip Select low */
	FLASH_CS_DOWN;

	/* Send "RDID " instruction */
	SPI_ReadWriteByte(AT45DB_RD_ID);

	/* Read a byte from the FLASH */
	Temp0 = SPI_ReadWriteByte(AT45DB_DUMMY_BYTE);

	/* Read a byte from the FLASH */
	Temp1 = SPI_ReadWriteByte(AT45DB_DUMMY_BYTE);

	/* Read a byte from the FLASH */
	Temp2 = SPI_ReadWriteByte(AT45DB_DUMMY_BYTE);

	/* Deselect the FLASH: Chip Select high */
	FLASH_CS_UP;

	Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

	return Temp;
}

void AT45DB_Write_Page(void* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	uint8_t *buf = pBuffer;
	uint16_t i;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_PP_THRU_BUF1);
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) WriteAddr);
	for (i = 0; i < NumByteToWrite; i++)
		SPI_ReadWriteByte(buf[i]);
	FLASH_CS_UP;
	AT45DB_Wait_Busy();
}

void AT45DB_MemoryToBuf(uint32_t WriteAddr, uint8_t p) {
	FLASH_CS_DOWN;

	SPI_ReadWriteByte(p == AT45DB_BUFFER1 ? AT45DB_MEM_TO_BUF1_TRANS : AT45DB_MEM_TO_BUF2_TRANS);

	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) WriteAddr);
	FLASH_CS_UP;
	AT45DB_Wait_Busy();
}

void AT45DB_DataToBuf(void* pBuffer, uint32_t WriteAddr,
		uint16_t NumByteToWrite, uint8_t p) {
	uint8_t *buf = pBuffer;
	uint16_t i;
	FLASH_CS_DOWN;

	SPI_ReadWriteByte(p == 1 ? AT45DB_BUFF1_WRITE : AT45DB_BUFF2_WRITE);

	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) WriteAddr);
	for (i = 0; i < NumByteToWrite; ++i) {
		SPI_ReadWriteByte(buf[i]);
	}
	FLASH_CS_UP;
}

void AT45DB_BufToMemWrite(uint32_t WriteAddr, uint8_t p) {
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(
			p == AT45DB_BUFFER1 ? AT45DB_BUF1_TO_MEM_WERASE : AT45DB_BUF2_TO_MEM_WERASE);
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) WriteAddr);
	FLASH_CS_UP;
	AT45DB_Wait_Busy();
}

uint32_t AT45DB_MemToBufCompare(uint32_t WriteAddr, uint8_t p) {
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(
			p == AT45DB_BUFFER1 ? AT45DB_MEM_TO_BUF1_COMP : AT45DB_MEM_TO_BUF2_COMP);
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((WriteAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) WriteAddr);
	FLASH_CS_UP;
	AT45DB_Wait_Busy();
	return AT45DB_ReadSR() & AT45DB_COMP_BIT ? 0 : 1;
}

void AT45DB_Write(void* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite) {
	uint16_t secoff;
	uint16_t secremain;

	while (NumByteToWrite) {
		AT45DB_MemoryToBuf(WriteAddr, AT45DB_BUFFER1);

		secoff = WriteAddr % 512;
		secremain = 512 - secoff;
		if (secremain > NumByteToWrite)
			secremain = NumByteToWrite;

		AT45DB_DataToBuf(pBuffer, WriteAddr, secremain, AT45DB_BUFFER1);
		if (!AT45DB_MemToBufCompare(WriteAddr, AT45DB_BUFFER1)) {
			AT45DB_BufToMemWrite(WriteAddr, AT45DB_BUFFER1);
		}
		WriteAddr += secremain;
		NumByteToWrite -= secremain;
		pBuffer +=secremain;
	}
}

void AT45DB_PageErase(uint32_t PageAddr) {
	/* page Erase */
	/* Select the FLASH: Chip Select low */
	FLASH_CS_DOWN;
	/* Send Sector Erase instruction */
	SPI_ReadWriteByte(AT45DB_PAGE_ERASE);
	/* Send SectorAddr high nibble address byte */
	SPI_ReadWriteByte((PageAddr & 0xFF0000) >> 16);
	/* Send SectorAddr medium nibble address byte */
	SPI_ReadWriteByte((PageAddr & 0xFF00) >> 8);
	/* Send SectorAddr low nibble address byte */
	SPI_ReadWriteByte(PageAddr & 0xFF);
	/* Deselect the FLASH: Chip Select high */
	FLASH_CS_UP;

	/* Wait the end of Flash writing */
	AT45DB_Wait_Busy();
}

void AT45DB_SectorErase(uint32_t Dst_Addr) {
	Dst_Addr *= 4096;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_SECTOR_ERASE);
	SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 16));
	SPI_ReadWriteByte((uint8_t)((Dst_Addr) >> 8));
	SPI_ReadWriteByte((uint8_t) Dst_Addr);
	FLASH_CS_UP;
	AT45DB_Wait_Busy();
}

void AT45DB_Wait_Busy(void) {
	while (!(AT45DB_ReadSR() & AT45DB_RDY_BIT)) {
		__asm("nop");
	}
}

void AT45DB_PowerDown() {
	uint8_t i;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_POWER_DOWN);
	FLASH_CS_UP;
	for (i = 5; i > 0; i--){
		__asm("nop");
	}
}

void AT45DB_WAKEUP() {
	uint8_t i;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_WAKE_UP);
	FLASH_CS_UP;
	for (i = 5; i > 0; i--){
		__asm("nop");
	}
}

//uint8_t SPI_ReadWriteByte(uint8_t TxData) {
//	uint8_t RxData = 0;
//
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){
//		__asm("nop");
//	}
//
//	SPI_I2S_SendData(SPI1, TxData);
//
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){
//		__asm("nop");
//	}
//	RxData = SPI_I2S_ReceiveData(SPI1);
//
//	return (uint8_t) RxData;
//}

void AT45DB_Read(void* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead) {
	uint8_t *buf = pBuffer;
	uint16_t i;
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_RD_DATA_LS);
	SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 16));
	SPI_ReadWriteByte((uint8_t)((ReadAddr) >> 8));
	SPI_ReadWriteByte((uint8_t) ReadAddr);
	for (i = 0; i < NumByteToRead; i++) {
		buf[i] = SPI_ReadWriteByte(0XFF);
	}
	FLASH_CS_UP;
}

void AT45DB_ChipErase(void) {
	FLASH_CS_DOWN;
	SPI_ReadWriteByte(AT45DB_CHIP_ERASE_BYTE1);
	SPI_ReadWriteByte(AT45DB_CHIP_ERASE_BYTE2);
	SPI_ReadWriteByte(AT45DB_CHIP_ERASE_BYTE3);
	SPI_ReadWriteByte(AT45DB_CHIP_ERASE_BYTE4);
	FLASH_CS_UP;

	AT45DB_Wait_Busy();
}
