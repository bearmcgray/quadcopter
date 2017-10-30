/**
 *****************************************************************************
 * @title   main.c
 * @author  bear
 * @date    2014i14
 * @brief  fossil from home
 *******************************************************************************
*/

#include "stm32f4xx.h"
#include "led.h"
#include "uart_log.h"
#include <math.h>
#include "bmp180.h"
#include "mpu6050.h"
#include "i2c_bus.h"
#include "pwm_driver.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "pid.h"
#include "analog_drv.h"
#include "spi_bus.h"
#include "a7105_drv.h"
#include "at45db161.h"
#include "hmc5883.h"
#include "wake.h"
#include "cbuff.h"
#include "MadgwickAHRS.h"

//#define PITCH_OFFSET (0.65)
//#define ROLL_OFFSET (0.6)

//#define comp_coef (0.01)

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Delay(__IO uint32_t nCount);
/* Private functions ---------------------------------------------------------*/



//float KP = 110; //5.5; //26.0; //25 27/6/12
//float KI = 0; //10; //0.9; //200.0; //85 5/6/12
//float KD = 20; //0.9; //7; //7 27/6/12
//float yaw_mul = 20; //0.9; //7; //7 27/6/12

//float KP_in = 190;
//float KI_in = 20;
//float KD_in = 30;

float pitch = 0;
float roll = 0;
float yaw = 0;
float target_pitch = 0;
float target_roll = 0;
float target_yaw = 0;

tled led;
TEULER_Struct eu;

tpid pitch_pid;
tpid roll_pid;
tpid yaw_pid;

tHMC5883_data gau;
AHRS_Struct ahrs;

wake_data w;
unsigned char wake_out_buff[256];
unsigned char wake_in_buff[256];

cbuff_data cb;

int pwm1 = 0, pwm2 = 0, pwm3 = 0, pwm4 = 0;
int pwm1_comp = 0, pwm2_comp = 0, pwm3_comp = 0, pwm4_comp = 0;

char rf_data[21];
short thrust = 0;

tmiddler m;
tBMP180_calibration_data bar;

volatile s32 gauss_maxx = 0;
volatile s32 gauss_minx = 0;

volatile s32 gauss_maxy = 0;
volatile s32 gauss_miny = 0;

volatile s32 gauss_maxz = 0;
volatile s32 gauss_minz = 0;

int curve(int p){
	return p;//curve disabled now, it seems work unstable
	int tmp;
	if(p>1120){
		tmp = (p*612+1084792)/1000;
	}else{
		tmp = (p*1581)/1000;
	}
	return tmp;
}

typedef struct{
	unsigned int magic;
	float dt;
	float mag_dt;

	float kp;
	float ki;
	float kd;

	float yaw_kp;
	float yaw_ki;
	float yaw_kd;

	float trick;
	float comp_coef;
	float pr_coef;
	float gyro_coef;

	float pitch_offset;
	float roll_offset;
	float yaw_offset;

	float res0[10];

	int gauss_xoffset;
	int gauss_yoffset;
	int gauss_zoffset;

	int pitch_pid_max;
	int pitch_pid_min;

	int roll_pid_max;
	int roll_pid_min;

	int yaw_pid_max;
	int yaw_pid_min;


	int res1[10];
}storage;

storage tmp_store;
storage default_store;
storage *store;



//#define USART_IS (1)//uart
#define USART_IS (3)//cp2102
#if USART_IS == 1
	const USART_TypeDef* USART = USART1;
#elif USART_IS == 3
	const USART_TypeDef* USART = USART3;
#endif

int bc=0;

#define ONLY_GYRO
#define NO_BAROMETR
//#define NO_MAGNITOMETR
#define USE_QUAT

int main(void) {
	SystemInit();

	middle_init(&m, 8);
	WAKE_init(&w);
	cbuff_init(&cb);

	SPI_Bus_Init();

	default_store.magic = 0x55aa6996;
	default_store.dt = 1.0/200.0;
	default_store.mag_dt = 1.0/75.0;

	default_store.kp = 6.0;
	default_store.ki = 11.0;
	default_store.kd = 0.7;

	default_store.yaw_kp = 100;
	default_store.yaw_ki = 0;
	default_store.yaw_kd = 0;

	default_store.trick = 3.5;
	default_store.comp_coef = 0.18;//aka 0.01
	default_store.pr_coef = 0.3;
	default_store.gyro_coef = 0.2;


	default_store.pitch_offset = 0.0;
	default_store.roll_offset = 0.0;
	default_store.yaw_offset = 0.0;

	default_store.gauss_xoffset = -60;
	default_store.gauss_yoffset = +400;
	default_store.gauss_zoffset = -288;

	default_store.pitch_pid_max = 400;
	default_store.pitch_pid_min = -400;

	default_store.roll_pid_max = 400;
	default_store.roll_pid_min = -400;

	default_store.yaw_pid_max = 400;
	default_store.yaw_pid_min = -400;

	unsigned int tmp = AT45DB_ReadID();
	if (tmp==0x1f2600){
		AT45DB_Read(&tmp_store,0,sizeof(storage));
		if(tmp_store.magic!=0x55aa6996){

			AT45DB_ChipErase();
			AT45DB_Write(&default_store,0,sizeof(storage));
			store = &default_store;

		}else{
			store = &tmp_store;
		}
	}else{
		store = &default_store;
	}


	setzero(&gau, sizeof(tHMC5883_data));
	gau.GAUSS_XOFFSET = store->gauss_xoffset;
	gau.GAUSS_YOFFSET = store->gauss_xoffset;
	gau.GAUSS_ZOFFSET = store->gauss_xoffset;

	PID_Init(&pitch_pid);
	PID_SetCoefs(&pitch_pid, store->kp, store->ki, store->kd, store->dt);
	PID_SetLimits(&pitch_pid, store->pitch_pid_max, store->pitch_pid_min);

	PID_Init(&roll_pid);
	PID_SetCoefs(&roll_pid, store->kp, store->ki, store->kd, store->dt);
	PID_SetLimits(&roll_pid, store->roll_pid_max, store->roll_pid_min);

	PID_Init(&yaw_pid);
	PID_SetCoefs(&yaw_pid, store->yaw_kp, store->yaw_ki, store->yaw_kd, store->mag_dt);
	PID_SetLimits(&yaw_pid, store->yaw_pid_max, store->yaw_pid_min);

	for (int i = 0; i < 0x3ffff; ++i) {
		__asm("nop");
	}

	I2C_Bus_Init();
	Analog_Init();

#ifndef NO_BAROMETR
	I2C_BMP180_GetCalibrationData(&bar);
#endif

	Init_MPU6050(&eu, store->pr_coef);
	//Calibrate_Gyros(&eu);

	usart_init(USART);
	LED_Init(&led);
	LED_On(&led, LED0 | LED1);

	PWM_PinsInit();

	//PWM_Update_All(100,100,100,100);
	//while(1);
#ifndef NO_MAGNITOMETR
	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_A, 0x30);
	I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_CONFIG_B, 0x00);
#endif
	TIM2_200Hz_Configuration(/*store->dt*/);//todo
	NVIC_ForTIM2_Configuration();

	int cntr = 0;

	Init_AHRS(&ahrs, 0.33, 50);
	A7105_Initialize();

	A7105_WriteID(0x5475c52a);
	//do {
	//	cntr = AT45DB_ReadID();
	//} while (cntr != 0x1f2600);

	//do {
	//	cntr = A7105_ReadID();
	//} while (cntr != 0x5475c52a);


	A7105_SwitchChanel(-1);

	int tg = 100;
	int rf_fail_cntr = 0;

	/*while(2){
		__asm("nop");
	}
	 */
	while (1) {

		if (A7105_WaitWTF()) {//todo
			char a = A7105_ReadReg(0x00);
			if ((a & 0x60) == 0x00) {
				cntr++;
				A7105_ReadData((u8*) &rf_data, 21);

				//if(cntr>100){
				//	cntr=0;
				//	LED_Toggle(&led, LED3);
				//}

				if (cntr > tg) {
					cntr = 0;
					LED_Toggle(&led, LED0);
					LED_Toggle(&led, LED1);

					//if(tg==104 && cntr == 0 )tg=400;else
					if (tg == 13 && cntr == 0)
						tg = 100;
					else if (tg == 12 && cntr == 0)
						tg = 13;
					else if (tg == 11 && cntr == 0)
						tg = 12;
					else if (tg == 100 && cntr == 0)
						tg = 11;

				}
				rf_fail_cntr = 0;
				int t = ((rf_data[8] << 8) + rf_data[7]) - 1077;
				thrust = (t) * 3 + (t >> 1); ///855*2500 ;

				target_pitch = -40 * (((rf_data[10] << 8) + rf_data[9]) - 1528)
								/ 453;
				target_roll = 40 * (((rf_data[6] << 8) + rf_data[5]) - 1510)
								/ 424;
				target_yaw = -200 * (((rf_data[12] << 8) + rf_data[11]) - 1502)
								/ 357;

				//chanel 5
				//KP_in = 250.0 * (((rf_data[14] << 8) + rf_data[13]) - 1000)/ 1000.0 - 17;
				//chanel 6
				//KI_in = 250.0 * (((rf_data[16] << 8) + rf_data[15]) - 1000)/ 1000.0 - 14;
				//chanel 7
				//KD_in = 250.0 * (((rf_data[18] << 8) + rf_data[17]) - 1000)/ 1000.0 - 17;

				//KP = KP_in;
				//KI = KI_in;
				//KD = KD_in;

				//int wlen = WAKE_pack(wake_out_buff,0,25,14,rf_data+5);
				//wr_buf(USART, wake_out_buff,wlen);
				//wr_group(KP,KI,KD);

				//PID_SetCoefs(&pitch_pid, KP/3, KI/8, KD/8, 1.0 / 200.0);
				//PID_SetCoefs(&roll_pid,  KP/3, KI/8, KD/8, 1.0 / 200.0);
				//PID_SetCoefs(&roll_pid, KP, KI, KD, 1.0 / 75.0);

				//yaw_mul = KI_in;
				A7105_SwitchChanel(1);

			}
		} else {
			if (rf_fail_cntr >= 10) {
				thrust = -1;
			} else {
				rf_fail_cntr++;
			}
			A7105_SwitchChanel(2);

		}
		char tmpchar=0;
		while(cbuff_get(&cb,&tmpchar)){
			int pack_len=0;
			if(pack_len=WAKE_bytein(&w,tmpchar,wake_in_buff)){
				LED_Toggle(&led,LED2);
				if(wake_in_buff[2]==69){
					char c[]="response";
					int kp,ki,kd;
					int l = WAKE_pack(wake_out_buff,0,70,8,c);
					kp = (wake_in_buff[ 7]<<24)|(wake_in_buff[ 6]<<16)|(wake_in_buff[ 5]<< 8)|(wake_in_buff[ 4]<<0);
					ki = (wake_in_buff[11]<<24)|(wake_in_buff[10]<<16)|(wake_in_buff[ 9]<< 8)|(wake_in_buff[ 8]<<0);
					kd = (wake_in_buff[15]<<24)|(wake_in_buff[14]<<16)|(wake_in_buff[13]<< 8)|(wake_in_buff[12]<<0);
					float KP_in = ((float)kp)/1000;
					float KI_in = ((float)ki)/1000;
					float KD_in = ((float)kd)/1000;
					float dt  = 1.0/200.0;

					PID_SetCoefs(&pitch_pid, KP_in, KI_in, KD_in, dt);
					PID_SetCoefs(&roll_pid,  KP_in, KI_in, KD_in, dt);

					wr_buf(USART,wake_out_buff,l);
				}
			}
		}
	}
}

/**
 * @brief  Delay Function.
 * @param  nCount:specifies the Delay time length.
 * @retval None
 */
void Delay(__IO uint32_t nCount) {
	while (nCount--) {
	}
}


volatile u32 gauss_skip = 0;
volatile u32 gauss_arm = 0;

float prev_pitch;
float prev_roll;
float dpitch;
float droll;
//#define COMP_FOR_PID 0.2

void TIM2_IRQHandler(void) {

	//ITStatus is =  USART_GetITStatus(USART3, USART_IT_RXNE);
	//USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);

	LED_Toggle(&led,LED3);
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//handler here

		//wr_str("11\r");
		//wr_int( 255*sin((k++)*0.01) );
		//I2C_GetTempPress(&bar);
		//Get_Gyro_Rates(&eu);

		//wr_int(bar.Temperature);

		Get_Accel_Values(&eu);
		Get_Accel_Angles(&eu);
		Get_Gyro_Rates(&eu);
		bc++;
		if (bc==20){
			bc = 0;
			LED_Toggle(&led,LED0);
		}

#ifndef NO_MAGNITOMETR
		I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_MODE, 0x01);
		Get_Gauss_Values(&gau);
#else
		wr_long_group9(USART,
			eu.ACCEL_XOUT,eu.ACCEL_YOUT,eu.ACCEL_ZOUT,
			eu.GYRO_XRATE,eu.GYRO_YRATE,eu.GYRO_ZRATE,
			0,0,0);
#endif

		float gc = 1./57.3/16.4;
		float ac = 1./32768.*9.8*4;

		float gx = eu.GYRO_XRATE*gc;
		float gy = eu.GYRO_YRATE*gc;
		float gz = eu.GYRO_ZRATE*gc;

		float ax = eu.ACCEL_XOUT*ac;
		float ay = eu.ACCEL_YOUT*ac;
		float az = eu.ACCEL_ZOUT*ac;

		float mx = -(gau.GAUSS_XOUT+230);
		float my = -(gau.GAUSS_YOUT+100);//must be negativ but now wrong c code in hmc5883.c line 81
		float mz = (gau.GAUSS_ZOUT+100);//must be positive but now wrong c code in hmc5883.c line 82
		MadgwickAHRSupdate(&ahrs,gx,gy,gz,ax,ay,az,mx,my,mz);

#ifdef USE_QUAT
		wr_long_group(USART,
				ahrs.q0*1000, ahrs.q1*1000, ahrs.q2*1000, ahrs.q3*1000,
				0, 0);
#else
		wr_long_group9(USART,
				eu.ACCEL_XOUT,eu.ACCEL_YOUT,eu.ACCEL_ZOUT,
				eu.GYRO_XRATE,eu.GYRO_YRATE,eu.GYRO_ZRATE,
				gau.GAUSS_XOUT,gau.GAUSS_YOUT,gau.GAUSS_ZOUT);

#endif

		getEuler(&ahrs);


#if 0
		if(gauss_skip++>=5){
			gauss_skip = 0;
			I2C_Bus_ByteWrite(I2C_HW_HMC5883_ADDR, HMC5883_RA_MODE, 0x01);
			gauss_arm = 1;
		}
#endif
#if 0
		if(0){
			if(  (I2C_Bus_ByteRead(I2C_HW_HMC5883_ADDR, HMC5883_RA_STATUS )&0x01)==0x01 ){
				Get_Gauss_Values(&gau);

				heading_tilt_comp(&gau,pitch,roll);
			}
		}
#endif
		pitch = (1-store->comp_coef) * (pitch + eu.GYRO_YRATE * pitch_pid.dt / store->trick)
						+ (store->comp_coef) * (eu.ACCEL_YANGLE);

		roll = (1-store->comp_coef) * (roll + eu.GYRO_XRATE * roll_pid.dt / store->trick)
						+ (store->comp_coef) * (eu.ACCEL_XANGLE);

		yaw = (eu.GYRO_ZRATE + target_yaw) * store->yaw_kp;

		//--------------------
		/////////wr_group(eu.GYRO_ZRATE/*gau.heading*/, pitch, roll);
		//wr_group(USART, pitch, eu.ACCEL_YANGLE, eu.GYRO_YRATE);//use this

		if (yaw > 200)
			yaw = 200;

		if (yaw < -200)
			yaw = -200;
		//yaw=0;// debugging----------------------------------------

		//wr_group( pitch+PITCH_OFFSET, roll+ROLL_OFFSET, middle(&m,Analog_GetCurrent()));
		//wr_group( KP_in, KI_in, KD_in);
		//wr_group( target_pitch, target_roll, target_yaw);
		//wr_group(gau.GAUSS_XOUT, gau.GAUSS_YOUT, gau.GAUSS_ZOUT);

		if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8) && thrust > 0) {
			LED_Off(&led, LED2);

			dpitch = (1 - store->gyro_coef)*dpitch + store->gyro_coef * eu.GYRO_YRATE;
			droll  = (1 - store->gyro_coef)*droll  + store->gyro_coef * eu.GYRO_XRATE;

			PID_Compute(&pitch_pid, target_pitch, pitch, dpitch);
			PID_Compute(&roll_pid, target_roll, roll,	droll);
			PID_Compute(&yaw_pid, target_yaw, gau.heading,	eu.GYRO_ZRATE);

			//wr_group(pitch, PID_GetOutput(&pitch_pid), pitch_pid.integral_tmp);//use this


			//		thrust	pitch					roll					yaw

			pwm1 = thrust - PID_GetOutput(&pitch_pid)
								  - PID_GetOutput(&roll_pid)
								  +yaw;// + PID_GetOutput(&yaw_pid);
			pwm1 = curve(pwm1);

			pwm2 = thrust + PID_GetOutput(&pitch_pid)
					    		  - PID_GetOutput(&roll_pid)
					    		  -yaw;//- PID_GetOutput(&yaw_pid);
			pwm2 = curve(pwm2);

			pwm3 = thrust + PID_GetOutput(&pitch_pid)
					    		  + PID_GetOutput(&roll_pid)
					    		  +yaw;//+ PID_GetOutput(&yaw_pid);
			pwm3 = curve(pwm3);

			pwm4 = thrust - PID_GetOutput(&pitch_pid)
					    		  + PID_GetOutput(&roll_pid)
					    		  -yaw;//- PID_GetOutput(&yaw_pid);
			pwm4 = curve(pwm4);
			if (pwm1 > 2799) {
				pwm1 = 2799;
				LED_On(&led, LED2);
				//pwm1_comp = pwm1 - 2799;
				//pwm3 -= pwm1_comp;
			}
			if (pwm2 > 2799) {
				pwm2 = 2799;
				LED_On(&led, LED2);
				//pwm2_comp = pwm2 - 2799;
				//pwm4 -= pwm2_comp;
			}
			if (pwm3 > 2799) {
				pwm3 = 2799;
				LED_On(&led, LED2);
				//pwm3_comp = pwm3 - 2799;
				//pwm1 -= pwm3_comp;
			}
			if (pwm4 > 2799) {
				pwm4 = 2799;
				LED_On(&led, LED2);
				//pwm4_comp = pwm4 - 2799;
				//pwm2 -= pwm4_comp;
			}

			if (pwm1 < 0) {
				pwm1 = 0;
			}
			if (pwm2 < 0) {
				pwm2 = 0;
			}
			if (pwm3 < 0) {
				pwm3 = 0;
			}
			if (pwm4 < 0) {
				pwm4 = 0;
			}

			PWM_Update_All(pwm1,pwm2,pwm3,pwm4);

		} else {
			//LED_Off(&led, LED2);
			PWM_Update_All(1,1,1,1);
		}
	}
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
}

#if USART_IS == 1
void USART1_IRQHandler(void){
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		char t = USART1->DR;
		cbuff_put(&cb,t);
	}
}
#elif USART_IS == 3

// this is the interrupt request handler (IRQ) for ALL USART3 interrupts
void USART3_IRQHandler(void){
	// check if the USART3 receive interrupt flag was set
	if( USART_GetITStatus(USART3, USART_IT_RXNE) ){
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);

		char t = USART3->DR; // the character from the USART3 data register is saved in t
		cbuff_put(&cb,t);
	}
}
#endif

