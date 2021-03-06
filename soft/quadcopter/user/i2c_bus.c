#include "i2c_bus.h"
#include "stm32f4xx_i2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void I2C_Bus_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7
	 * 2. SCL on PB8 and SDA on PB9
	 */
	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1); // SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz; // set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP; // enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct); // init GPIOB

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = I2C_Speed;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x01; // own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable; // disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses


	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
	I2C_Init(I2C1, &I2C_InitStruct); // init I2C1

	while( I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY) ){
		I2C_SoftwareResetCmd(I2C1,ENABLE);
		I2C_SoftwareResetCmd(I2C1,DISABLE);
	}



}

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction) {
	// wait until I2C1 is not busy anymore
	while (I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY))
		;

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT))
		;

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if (direction == I2C_Direction_Transmitter) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
			;
	} else if (direction == I2C_Direction_Receiver) {
		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
			;
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
		;
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx) {
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx) {
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED))
		;
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx) {
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_Bus_ByteWrite(unsigned char slaveAddr, unsigned char writeAddr,
		unsigned char data) {

	I2C_start(I2C1, slaveAddr, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, writeAddr); // write one byte to the slave
	I2C_write(I2C1, data); // write one byte to the slave
	I2C_stop(I2C1); // stop the transmission

}

/**
 * @brief Reads a block of data from the MPU6050.
 * @param slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
 * @param pBuffer : pointer to the buffer that receives the data read from the MPU6050.
 * @param readAddr : MPU6050's internal address to read from.
 * @param NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1 only for the Mgnetometer readinf).
 * @return None
 */

unsigned char I2C_Bus_ByteRead(unsigned char slaveAddr, unsigned char readAddr) {

	unsigned char tmp = 0;
	I2C_start(I2C1, slaveAddr, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, readAddr); // write one byte to the slave
	I2C_stop(I2C1); // stop the transmission
	I2C_start(I2C1, slaveAddr, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	tmp = I2C_read_nack(I2C1); // read one byte and request another byte
	I2C_stop(I2C1); // stop the transmission
	return tmp;
}
