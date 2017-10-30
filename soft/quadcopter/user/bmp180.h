#ifndef __BMP180_H
#define __BMP180_H

#define I2C_HW_BMP180_ADDR    0xEE

#define BMP180_START_TEMP_OSS0  0x2E

#define BMP180_START_PRESS_OSS0  0x34
#define BMP180_START_PRESS_OSS1  0x74
#define BMP180_START_PRESS_OSS2  0xB4
#define BMP180_START_PRESS_OSS3  0xF4

#define I2C_REG_AC1_MSB_ADDR  0xAA
#define I2C_REG_AC1_LSB_ADDR  0xAB
#define I2C_REG_AC2_MSB_ADDR  0xAC
#define I2C_REG_AC2_LSB_ADDR  0xAD
#define I2C_REG_AC3_MSB_ADDR  0xAE
#define I2C_REG_AC3_LSB_ADDR  0xAF
#define I2C_REG_AC4_MSB_ADDR  0xB0
#define I2C_REG_AC4_LSB_ADDR  0xB1
#define I2C_REG_AC5_MSB_ADDR  0xB2
#define I2C_REG_AC5_LSB_ADDR  0xB3
#define I2C_REG_AC6_MSB_ADDR  0xB4
#define I2C_REG_AC6_LSB_ADDR  0xB5

#define I2C_REG_B1_MSB_ADDR  0xB6
#define I2C_REG_B1_LSB_ADDR  0xB7
#define I2C_REG_B2_MSB_ADDR  0xB8
#define I2C_REG_B2_LSB_ADDR  0xB9

#define I2C_REG_MB_MSB_ADDR  0xBA
#define I2C_REG_MB_LSB_ADDR  0xBB
#define I2C_REG_MC_MSB_ADDR  0xBC
#define I2C_REG_MC_LSB_ADDR  0xBD
#define I2C_REG_MD_MSB_ADDR  0xBE
#define I2C_REG_MD_LSB_ADDR  0xBF

#define I2C_REG_RESULT_MSB_ADDR  0xF6
#define I2C_REG_RESULT_LSB_ADDR  0xF7
#define I2C_REG_RESULT_XLSB_ADDR  0xF8

#define I2C_REG_MEASUREMENT_ADDR  0xF4

#define BMP180_START_TEMP_OSS0  0x2E

#define BMP180_START_PRESS_OSS0  0x34
#define BMP180_START_PRESS_OSS1  0x74
#define BMP180_START_PRESS_OSS2  0xB4
#define BMP180_START_PRESS_OSS3  0xF4

#define BMP180_CONV_READY  5

typedef struct {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;
	long Temperature;
	long Pressure;
	long Altitude;
} tBMP180_calibration_data;

void I2C_BMP180_GetCalibrationData(tBMP180_calibration_data *data);
void I2C_GetTempPress(tBMP180_calibration_data *data);


#endif //__BMP180_H
