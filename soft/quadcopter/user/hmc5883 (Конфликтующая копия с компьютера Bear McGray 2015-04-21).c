#include"hmc5883.h"
#include "math.h"
#include "i2c_bus.h"

float heading_tilt_comp(tHMC5883_data* dat, float pitch,
		float roll) {
	float theta = pitch / 57.295;
	float phi = roll / 57.295;

	float Yh = dat->GAUSS_ZOUT*sin(phi  ) - dat->GAUSS_YOUT*cos(phi  );
	float Xh = dat->GAUSS_XOUT*cos(theta) + dat->GAUSS_YOUT*sin(theta)*sin(phi) + dat->GAUSS_ZOUT*sin(theta)*cos(phi);

	dat->heading = atan2(Yh, Xh) * 57.295;
	return dat->heading;
}

void offset_search(tHMC5883_data* dat) {
	tHMC5883_data data;
	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_A, 0x32);
	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_B, 0x00);

	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_MODE, 0x01);
	while (!((I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR, HMC5883_RA_STATUS) & 0x01)
			== 0x01))
		;

	Get_Gauss_Values(&data);
	short pos_x_offs = data.GAUSS_XOUT;
	short pos_y_offs = data.GAUSS_YOUT;
	short pos_z_offs = data.GAUSS_ZOUT;

	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_A, 0x31);
	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_MODE, 0x01);
	while (!((I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR, HMC5883_RA_STATUS) & 0x01)
			== 0x01))
		;

	Get_Gauss_Values(&data);
	short neg_x_offs = data.GAUSS_XOUT;
	short neg_y_offs = data.GAUSS_YOUT;
	short neg_z_offs = data.GAUSS_ZOUT;

	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_A, 0x30);

	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_MODE, 0x01);
	while (!((I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR, HMC5883_RA_STATUS) & 0x01)
			== 0x01))
		;

	Get_Gauss_Values(&data);
	short x = data.GAUSS_XOUT;
	short y = data.GAUSS_YOUT;
	short z = data.GAUSS_ZOUT;
}

void Get_Gauss_Values(tHMC5883_data* dat) {
	char GAUSS_XOUT_H;
	char GAUSS_XOUT_L;
	char GAUSS_YOUT_H;
	char GAUSS_YOUT_L;
	char GAUSS_ZOUT_H;
	char GAUSS_ZOUT_L;


	//while( !( (I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR, HMC5883_RA_STATUS )&0x01)==0x01 ) );

	GAUSS_XOUT_H = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_XOUT_H);
	GAUSS_XOUT_L = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_XOUT_L);
	GAUSS_YOUT_H = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_YOUT_H);
	GAUSS_YOUT_L = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_YOUT_L);
	GAUSS_ZOUT_H = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_ZOUT_H);
	GAUSS_ZOUT_L = I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR,
			HMC5883_RA_GAUSS_ZOUT_L);

	dat->GAUSS_XOUT =  ( ((GAUSS_XOUT_H << 8) | GAUSS_XOUT_L) + dat->GAUSS_XOFFSET);
	dat->GAUSS_YOUT = -( ((GAUSS_YOUT_H << 8) | GAUSS_YOUT_L) + dat->GAUSS_YOFFSET);
	dat->GAUSS_ZOUT = -( ((GAUSS_ZOUT_H << 8) | GAUSS_ZOUT_L) + dat->GAUSS_ZOFFSET);
}
