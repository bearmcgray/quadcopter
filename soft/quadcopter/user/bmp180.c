#include "bmp180.h"
#include "i2c_bus.h"

void I2C_GetTempPress(tBMP180_calibration_data *data) {
	char tmp_msb, tmp_lsb, tmp_xlsb;
	long ut;
	long up;
	unsigned long b4;
	long x1, x2, x3, b3, b5, b6, b7, p;

	I2C_Bus_ByteWrite(I2C_HW_BMP180_ADDR, I2C_REG_MEASUREMENT_ADDR,
			BMP180_START_TEMP_OSS0);
	//for(int i = 0; i<0x3fffff;++i);
	while (I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MEASUREMENT_ADDR)
			& (1 << BMP180_CONV_READY))
		;
	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_RESULT_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_RESULT_LSB_ADDR);
	ut = (tmp_msb << 8) | tmp_lsb;

	I2C_Bus_ByteWrite(I2C_HW_BMP180_ADDR, I2C_REG_MEASUREMENT_ADDR,
			BMP180_START_PRESS_OSS3);
	while (I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MEASUREMENT_ADDR)
			& (1 << BMP180_CONV_READY))
		;
	//for(int i = 0; i<0x3fffff;++i);
	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_RESULT_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_RESULT_LSB_ADDR);
	tmp_xlsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_RESULT_XLSB_ADDR);
	up = ((tmp_msb << 16) | (tmp_lsb << 8) | tmp_xlsb) >> (8 - 3);

	x1 = (ut - data->AC6) * data->AC5 / (1 << 15);
	x2 = data->MC * (1 << 11) / (x1 + data->MD);
	b5 = x1 + x2;
	data->Temperature = (b5 + 8) / (1 << 4);

	b6 = b5 - 4000;
	x1 = (data->B2 * b6 * b6 / (1 << 12)) / (1 << 11);
	x2 = data->AC2 * b6 / (1 << 11);
	x3 = x1 + x2;
	b3 = (((data->AC1 * 4 + x3) << 3) + 2) / 4; //3+2???
	x1 = data->AC3 * b6 / (1 << 13);
	x2 = (data->B1 * (b6 * b6 / (1 << 12))) / (1 << 16);
	x3 = (x1 + x2 + 2) / 4;
	b4 = data->AC4 * (unsigned long) (x3 + 32768) / (1 << 15);
	b7 = ((unsigned long) up - b3) * (50000 >> 3);
	if (b7 < 0x80000000) {
		p = (b7 * 2) / b4;
	} else {
		p = (b7 / b4) * 2;
	}
	x1 = (p / (1 << 8)) * (p / (1 << 8));
	x1 = (x1 * 3038) / (1 << 16);
	x2 = (-7357 * p) / (1 << 16);
	data->Pressure = p + (x1 + x2 + 3791) / (1 << 4);

	data->Altitude = 9890 - (data->Pressure - 90000) * 9890 / 11325;

}

void I2C_BMP180_GetCalibrationData(tBMP180_calibration_data *data) {
	char tmp_msb, tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC1_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC1_LSB_ADDR);
	data->AC1 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC2_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC2_LSB_ADDR);
	data->AC2 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC3_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC3_LSB_ADDR);
	data->AC3 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC4_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC4_LSB_ADDR);
	data->AC4 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC5_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC5_LSB_ADDR);
	data->AC5 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC6_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_AC6_LSB_ADDR);
	data->AC6 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_B1_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_B1_LSB_ADDR);
	data->B1 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_B2_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_B2_LSB_ADDR);
	data->B2 = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MB_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MB_LSB_ADDR);
	data->MB = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MC_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MC_LSB_ADDR);
	data->MC = (tmp_msb << 8) | tmp_lsb;

	tmp_msb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MD_MSB_ADDR);
	tmp_lsb = I2C_Bus_ByteRead(I2C_HW_BMP180_ADDR, I2C_REG_MD_LSB_ADDR);
	data->MD = (tmp_msb << 8) | tmp_lsb;
}
