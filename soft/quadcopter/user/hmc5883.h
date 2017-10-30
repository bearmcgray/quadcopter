#ifndef __HMC5883_H
#define __HMC5883_H

#define I2C_HW_HMC5883_ADDR 0x3c

#define HMC5883_RA_CONFIG_A		0x00
#define HMC5883_RA_CONFIG_B		0x01
#define HMC5883_RA_MODE			0x02

#define HMC5883_RA_GAUSS_XOUT_H	0x03
#define HMC5883_RA_GAUSS_XOUT_L	0x04
#define HMC5883_RA_GAUSS_ZOUT_H	0x05
#define HMC5883_RA_GAUSS_ZOUT_L	0x06
#define HMC5883_RA_GAUSS_YOUT_H	0x07
#define HMC5883_RA_GAUSS_YOUT_L	0x08

#define HMC5883_RA_STATUS		0x09
#define HMC5883_RA_ID_A			0x0a
#define HMC5883_RA_ID_B			0x0b
#define HMC5883_RA_ID_C			0x0c


typedef struct {
	short GAUSS_XOUT;
	short GAUSS_YOUT;
	short GAUSS_ZOUT;

	short GAUSS_XOFFSET;
	short GAUSS_YOFFSET;
	short GAUSS_ZOFFSET;

	float heading;
} tHMC5883_data;

float heading_tilt_comp(tHMC5883_data* dat, float phi_pitch_y,float theta_roll_x);
void offset_search(tHMC5883_data* dat);
void Get_Gauss_Values(tHMC5883_data* dat);


#endif //__HMC5883_H
