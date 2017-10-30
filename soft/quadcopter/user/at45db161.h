#ifndef __AT45DB161_H
#define __AT45DB161_H			    

#define AT45DB_RD_STATUS_REG	0xD7
#define AT45DB_RD_DATA_LS		0x03
#define AT45DB_PP_THRU_BUF1		0x82
#define AT45DB_PAGE_ERASE		0x81
#define AT45DB_SECTOR_ERASE		0x7C
#define AT45DB_POWER_DOWN		0xB9
#define AT45DB_WAKE_UP		0xAB
#define AT45DB_RD_ID		0x9F
#define AT45DB_MEM_TO_BUF1_TRANS	0x53
#define AT45DB_MEM_TO_BUF2_TRANS	0x55
#define AT45DB_MEM_TO_BUF1_COMP		0x60
#define AT45DB_MEM_TO_BUF2_COMP		0x61

#define AT45DB_BUFF1_WRITE	0x84
#define AT45DB_BUFF2_WRITE	0x87

#define AT45DB_BUF1_TO_MEM_WERASE	0x83
#define AT45DB_BUF2_TO_MEM_WERASE	0x86

#define AT45DB_RDY_BIT			0x80
#define AT45DB_COMP_BIT			0x40
#define AT45DB_PROT_BIT			0x02
#define AT45DB_PGSZ_BIT			0x01

#define AT45DB_CHIP_ERASE_BYTE1 0xc7
#define AT45DB_CHIP_ERASE_BYTE2 0x94
#define AT45DB_CHIP_ERASE_BYTE3 0x80
#define AT45DB_CHIP_ERASE_BYTE4 0x9a

#define AT45DB_PGSZ512_BYTE1 0x3D
#define AT45DB_PGSZ512_BYTE2 0x2A
#define AT45DB_PGSZ512_BYTE3 0x80
#define AT45DB_PGSZ512_BYTE4 0xA6

#define  AT45DB_DUMMY_BYTE  0xff
#define  AT45DB_PAGE_SIZE 	0x200

#define  AT45DB_BUFFER1  1
#define  AT45DB_BUFFER2  2

//#define AT45DB_CS  PBout(5)

//Bit 7     Bit 6 Bit 5 Bit 4 Bit 3 Bit 2 Bit 1   Bit 0
//RDY/BUSY  COMP  1     0     1     1     PROTECT PAGE_SIZE

void AT45DB_Init(void);
//uint8_t SPI_ReadWriteByte(uint8_t TxData);
uint8_t AT45DB_ReadSR(void);
void AT45DB_Wait_Busy(void);
void AT45DB_Repair512(void);
uint32_t AT45DB_ReadID(void);

void AT45DB_ChipErase(void);
void AT45DB_PageErase(uint32_t PageAddr);
void AT45DB_SectorErase(uint32_t Dst_Addr);

void AT45DB_Write_Page(void* pBuffer, uint32_t WriteAddr,
		uint16_t NumByteToWrite);

void AT45DB_Write(void* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);
void AT45DB_Read(void* pBuffer, uint32_t ReadAddr, uint16_t NumByteToRead);

void AT45DB_PowerDown(void);
void AT45DB_WAKEUP(void);

void AT45DB_MemoryToBuf(uint32_t WriteAddr, uint8_t p);
void AT45DB_DataToBuf(void* pBuffer, uint32_t WriteAddr,
		uint16_t NumByteToWrite, uint8_t p);
void AT45DB_BufToMemWrite(uint32_t WriteAddr, uint8_t p);
uint32_t AT45DB_MemToBufCompare(uint32_t WriteAddr, uint8_t p);

#endif

