#ifndef __SPI_BUS_H
#define __SPI_BUS_H

#define SPI_BUS SPI3

void SPI_Bus_Init();
unsigned char SPI_ReadWriteByte(unsigned char TxData);

#endif //__SPI_BUS_H
