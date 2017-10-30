#ifndef __I2C_BUS_H
#define __I2C_BUS_H

#define I2C_BUS               I2C1
#define I2C_GY87_CLK           RCC_APB1Periph_I2C1
#define I2C_GY87_GPIO          GPIOB
#define I2C_GY87_GPIO_CLK      RCC_APB2Periph_GPIOB
#define I2C_GY87_SCL           GPIO_Pin_6
#define I2C_GY87_SDA           GPIO_Pin_7

#define I2C_Speed              400000


void I2C_Bus_Init();
//void I2C_Bus_ByteWrite(unsigned char hw_addr, unsigned char reg_addr, unsigned char par);
//unsigned char I2C_Bus_ByteRead(unsigned char hw_addr, unsigned char reg_addr);

void I2C_Bus_ByteWrite(unsigned char slaveAddr, unsigned char writeAddr, unsigned char data);
///void I2C_Bus_ByteRead(unsigned char slaveAddr, unsigned char* pBuffer, unsigned char readAddr, unsigned short NumByteToRead);
unsigned char I2C_Bus_ByteRead(unsigned char slaveAddr,  unsigned char readAddr);

#endif //__I2C_BUS_H
