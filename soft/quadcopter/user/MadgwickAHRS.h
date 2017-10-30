//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

//extern volatile float beta;				// algorithm gain
//extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

typedef struct __ahrs{
	float beta;
	float q0;
	float q1;
	float q2;
	float q3;

	float samp_freq;

	float yaw;
	float pitch;
	float roll;

}AHRS_Struct;

//---------------------------------------------------------------------------------------------------
// Function declarations

void Init_AHRS(AHRS_Struct *a, float beta, float update_rate);

void MadgwickAHRSupdate(AHRS_Struct *a, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(AHRS_Struct *a,float gx, float gy, float gz, float ax, float ay, float az);
void getEuler(AHRS_Struct *a);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
