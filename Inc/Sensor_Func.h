#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "I2C_Com.h"

#ifndef SENSOR_FUNC
#define SENSOR_FUNC

typedef struct
{
	uint8_t ClockSource;
	uint8_t Gyro_Full_Scale;
	uint8_t Accel_Full_Scale;
	uint8_t CONFIG_DLPF;
	bool 		Sleep_Mode_Bit; 
	
}MPU_ConfigTypeDef;
//2- Clock Source ENUM
enum PM_CLKSEL_ENUM
{
	Internal_8MHz 	= 0x00,
	X_Axis_Ref			= 0x01,
	Y_Axis_Ref			= 0x02,
	Z_Axis_Ref			= 0x03,
	Ext_32_768KHz		= 0x04,
	Ext_19_2MHz			= 0x05,
	TIM_GENT_INREST	= 0x07
};
//3- Gyro Full Scale Range ENUM (deg/sec)
enum gyro_FullScale_ENUM
{
	FS_SEL_250 	= 0x00,
	FS_SEL_500 	= 0x01,
	FS_SEL_1000 = 0x02,
	FS_SEL_2000	= 0x03
};
//4- Accelerometer Full Scale Range ENUM (1g = 9.81m/s2)
enum accel_FullScale_ENUM
{
	AFS_SEL_2g	= 0x00,
	AFS_SEL_4g  = 0x01,
	AFS_SEL_8g  = 0x02,
	AFS_SEL_16g = 0x03
};
//5- Digital Low Pass Filter ENUM
enum DLPF_CFG_ENUM
{
	DLPF_260A_256G_Hz = 0x00,
	DLPF_184A_188G_Hz = 0x01,
	DLPF_94A_98G_Hz 	= 0x02,
	DLPF_44A_42G_Hz 	= 0x03,
	DLPF_21A_20G_Hz 	= 0x04,
	DLPF_10_Hz 				= 0x05,
	DLPF_5_Hz 				= 0x06
};
//6- e external Frame Synchronization ENUM
enum EXT_SYNC_SET_ENUM
{
	input_Disable = 0x00,
	TEMP_OUT_L		= 0x01,
	GYRO_XOUT_L		= 0x02,
	GYRO_YOUT_L		= 0x03,
	GYRO_ZOUT_L		= 0x04,
	ACCEL_XOUT_L	= 0x05,
	ACCEL_YOUT_L	= 0x06,
	ACCEL_ZOUT_L	= 0x07
};


void MPU6050_Config(MPU_ConfigTypeDef *config);
uint8_t MPU6050_Get_SMPRT_DIV(void);
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue);
uint8_t MPU6050_Get_FSYNC(void);
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync);
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef);
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef);
void HMC5883L_Config(void);
void HMC5883L_Get_Mag_RawData(RawData_Def *rawDef);
void MS5611_Get_Data(void);
void MS5611_Init(void);
int16_t MPU6050_Get_Temp(void);
#endif /* SENSOR_FUNC */
