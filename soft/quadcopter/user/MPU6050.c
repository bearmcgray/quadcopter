#include"MPU6050.h"
#include "i2c_bus.h"
#include <math.h>

#define rad2deg (57.295)

void Init_MPU6050(TEULER_Struct *data,float prc){
	data->pr_coef  = prc;
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_PWR_MGMT_1, 0x02);
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_SMPLRT_DIV, 0x01);
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_CONFIG, 0x02);
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG, 0x18);
	//char tmp = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_CONFIG);
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_CONFIG, 0x08);
	I2C_Bus_ByteWrite(I2C_HW_MPU6050_ADDR, MPU6050_RA_INT_PIN_CFG, 0x02);
}
float fXg,fYg,fZg;
//float alpha = 0.7;
void Get_Accel_Angles(TEULER_Struct *data)
{

	//ACCEL_XANGLE = 57.295*atan((float)ACCEL_YOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_XOUT,2)))*a + (1-a)*ACCEL_XANGLE;
	//ACCEL_YANGLE = 57.295*atan((float)-ACCEL_XOUT/ sqrt(pow((float)ACCEL_ZOUT,2)+pow((float)ACCEL_YOUT,2)))*a + (1-a)*ACCEL_YANGLE;

	//data->ACCEL_XANGLE = 57.295*atan((float)data->ACCEL_YOUT/ sqrt(pow((float)data->ACCEL_ZOUT,2)+pow((float)data->ACCEL_XOUT,2)));
	//data->ACCEL_YANGLE = 57.295*atan((float)-data->ACCEL_XOUT/ sqrt(pow((float)data->ACCEL_ZOUT,2)+pow((float)data->ACCEL_YOUT,2)));

	//data->ACCEL_XANGLE = atan2( data->ACCEL_YOUT, data->ACCEL_ZOUT);
	//data->ACCEL_YANGLE = atan2(-data->ACCEL_XOUT, data->ACCEL_YOUT*sin(data->ACCEL_XANGLE) + data->ACCEL_ZOUT*cos(data->ACCEL_XANGLE) )*rad2deg;
	//data->ACCEL_XANGLE *=rad2deg;
    //Low Pass Filter

    fXg = data->ACCEL_XOUT * (1 - data->pr_coef) + fXg * data->pr_coef;//invert coef
    fYg = data->ACCEL_YOUT * (1 - data->pr_coef) + fYg * data->pr_coef;
    fZg = data->ACCEL_ZOUT * (1 - data->pr_coef) + fZg * data->pr_coef;

    //Roll & Pitch Equations
    data->ACCEL_XANGLE  = -atan2(-fYg, fZg)*rad2deg;
    data->ACCEL_YANGLE = -atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*rad2deg;
}

void Get_Accel_Values(TEULER_Struct *data)
{

	char ACCEL_XOUT_H;
	char ACCEL_XOUT_L;
	char ACCEL_YOUT_H;
	char ACCEL_YOUT_L;
	char ACCEL_ZOUT_H;
	char ACCEL_ZOUT_L;

	ACCEL_XOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_H);
	ACCEL_XOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_XOUT_L);
	ACCEL_YOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_YOUT_H);
	ACCEL_YOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_YOUT_L);
	ACCEL_ZOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_ZOUT_H);
	ACCEL_ZOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_ACCEL_ZOUT_L);

	data->ACCEL_XOUT = ((ACCEL_XOUT_H<<8)|ACCEL_XOUT_L);
	data->ACCEL_YOUT = ((ACCEL_YOUT_H<<8)|ACCEL_YOUT_L);
	data->ACCEL_ZOUT = ((ACCEL_ZOUT_H<<8)|ACCEL_ZOUT_L);
}

void Calibrate_Gyros(TEULER_Struct *data)
{
	int x = 0;
	int GYRO_XOUT_OFFSET_1000SUM = 0;
	int GYRO_YOUT_OFFSET_1000SUM = 0;
	int GYRO_ZOUT_OFFSET_1000SUM = 0;

	char GYRO_XOUT_H;
	char GYRO_XOUT_L;
	char GYRO_YOUT_H;
	char GYRO_YOUT_L;
	char GYRO_ZOUT_H;
	char GYRO_ZOUT_L;


	for(x = 0; x<NUMCAL; x++)
	{
		GYRO_XOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H);
		GYRO_XOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_L);
		GYRO_YOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H);
		GYRO_YOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_L);
		GYRO_ZOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H);
		GYRO_ZOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_L);

		GYRO_XOUT_OFFSET_1000SUM += (short)((GYRO_XOUT_H<<8)|GYRO_XOUT_L);
		GYRO_YOUT_OFFSET_1000SUM += (short)((GYRO_YOUT_H<<8)|GYRO_YOUT_L);
		GYRO_ZOUT_OFFSET_1000SUM += (short)((GYRO_ZOUT_H<<8)|GYRO_ZOUT_L);

		//__delay_ms(1);
	}
	data->GYRO_XOUT_OFFSET = GYRO_XOUT_OFFSET_1000SUM/NUMCAL;
	data->GYRO_YOUT_OFFSET = GYRO_YOUT_OFFSET_1000SUM/NUMCAL;
	data->GYRO_ZOUT_OFFSET = GYRO_ZOUT_OFFSET_1000SUM/NUMCAL;
}

void Get_Gyro_Rates(TEULER_Struct *data)
{

	char GYRO_XOUT_H;
	char GYRO_XOUT_L;
	char GYRO_YOUT_H;
	char GYRO_YOUT_L;
	char GYRO_ZOUT_H;
	char GYRO_ZOUT_L;

	short GYRO_XOUT = 0;
	short GYRO_YOUT = 0;
	short GYRO_ZOUT = 0;

	GYRO_XOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_H);
	GYRO_XOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_XOUT_L);
	GYRO_YOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_H);
	GYRO_YOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_YOUT_L);
	GYRO_ZOUT_H = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_H);
	GYRO_ZOUT_L = I2C_Bus_ByteRead(I2C_HW_MPU6050_ADDR, MPU6050_RA_GYRO_ZOUT_L);

	GYRO_XOUT = ((GYRO_XOUT_H*256)+GYRO_XOUT_L);// - data->GYRO_XOUT_OFFSET;
	GYRO_YOUT = ((GYRO_YOUT_H*256)+GYRO_YOUT_L);// - data->GYRO_YOUT_OFFSET;
	GYRO_ZOUT = ((GYRO_ZOUT_H*256)+GYRO_ZOUT_L);// - data->GYRO_ZOUT_OFFSET;

	data->GYRO_XRATE = GYRO_XOUT;
	data->GYRO_YRATE = GYRO_YOUT;
	data->GYRO_ZRATE = GYRO_ZOUT;
	//data->GYRO_XRATE = -((float)GYRO_XOUT/32.8 - (float)data->GYRO_XOUT_OFFSET/32.8);///66.6;
	//data->GYRO_YRATE =  ((float)GYRO_YOUT/32.8 - (float)data->GYRO_YOUT_OFFSET/32.8);///66.6;
	//data->GYRO_ZRATE = -((float)GYRO_ZOUT/32.8 - (float)data->GYRO_ZOUT_OFFSET/32.8);///66.6;

}
