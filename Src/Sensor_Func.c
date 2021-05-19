#include "Sensor_Func.h"
#include "MPU6050_Add.h"
#include "OtherSensors_Add.h"
#include <math.h>


extern I2C_HandleTypeDef hi2c1;
//static int16_t GyroRW[3];
extern uint16_t C[7];

//1- MPU6050 Initialaztion Configuration 
void MPU6050_Config(MPU_ConfigTypeDef *config)
{
	uint8_t Buffer = 0;

	//Reset Device	
	I2C_Write8(MPU_ADDR, USER_CNT_REG, 0x00);
	I2C_Write8(MPU_ADDR, 0x37, 0x02);
	I2C_Write8(MPU_ADDR, PWR_MGMT_1_REG, 0x00);
	HAL_Delay(100);
	
	//Clock Source 
	Buffer = config ->ClockSource & 0x07; //change the 7th bits of register
	Buffer |= (config ->Sleep_Mode_Bit << 6) &0x40; // change only the 7th bit in the register
	I2C_Write8(MPU_ADDR, PWR_MGMT_1_REG, Buffer);
	HAL_Delay(100); // should wait 10ms after changeing the clock setting.
	
	//Set the Digital Low Pass Filter
	Buffer = 0;
	Buffer = config->CONFIG_DLPF & 0x07;
	I2C_Write8(MPU_ADDR, CONFIG_REG, Buffer);
	
	//Select the Gyroscope Full Scale Range
	Buffer = 0;
	Buffer = (config->Gyro_Full_Scale << 3) & 0x18;
	I2C_Write8(MPU_ADDR, GYRO_CONFIG_REG, Buffer);
	
	//Select the Accelerometer Full Scale Range 
	Buffer = 0; 
	Buffer = (config->Accel_Full_Scale << 3) & 0x18;
	I2C_Write8(MPU_ADDR, ACCEL_CONFIG_REG, Buffer);
	//Set SRD To Default
	MPU6050_Set_SMPRT_DIV(0x04);	
}

//2- Get Sample Rate Divider
uint8_t MPU6050_Get_SMPRT_DIV(void)
{
	uint8_t Buffer = 0;	
	I2C_Read(MPU_ADDR, SMPLRT_DIV_REG, &Buffer, 1);
	return Buffer;
}

//3- Set Sample Rate Divider
void MPU6050_Set_SMPRT_DIV(uint8_t SMPRTvalue)
{
	I2C_Write8(MPU_ADDR, SMPLRT_DIV_REG, SMPRTvalue);
}

//4- Get External Frame Sync.
uint8_t MPU6050_Get_FSYNC(void)
{
	uint8_t Buffer = 0;	
	I2C_Read(MPU_ADDR, CONFIG_REG, &Buffer, 1);
	Buffer &= 0x38; 
	return (Buffer>>3);
}

//5- Set External Frame Sync. 
void MPU6050_Set_FSYNC(enum EXT_SYNC_SET_ENUM ext_Sync)
{
	uint8_t Buffer = 0;
	I2C_Read(MPU_ADDR, CONFIG_REG, &Buffer,1);
	Buffer &= ~0x38;	
	Buffer |= (ext_Sync <<3); 
	I2C_Write8(MPU_ADDR, CONFIG_REG, Buffer);	
}

//6- Get Accel Raw Data
void MPU6050_Get_Accel_RawData(RawData_Def *rawDef)
{
	uint8_t i2cBuf[2] = {0};
	uint8_t AcceArr[6] = {0};
	
	I2C_Read(MPU_ADDR, INT_STATUS_REG, &i2cBuf[1], 1);
	if((i2cBuf[1]&&0x01))
	{
		//I2C_Read(ACCEL_XOUT_H_REG, AcceArr, 6);
		HAL_I2C_Mem_Read (&hi2c1, MPU_ADDR<<1, ACCEL_XOUT_H_REG, 1, AcceArr, 6, 100);
		//Accel Raw Data
		rawDef->x = ((AcceArr[0]<<8)|AcceArr[1]); // x-Axis Roll
		rawDef->y = ((AcceArr[2]<<8)|AcceArr[3]); // y-Axis Pitch
		rawDef->z = ((AcceArr[4]<<8)|AcceArr[5]); // z-Axis	Yaw
	}
}


//7- Get Gyro Raw Data
void MPU6050_Get_Gyro_RawData(RawData_Def *rawDef)
{
	uint8_t GyroArr[6];
	int16_t GyroRW[3];
	//Gyro Raw Data
	HAL_I2C_Mem_Read (&hi2c1, MPU_ADDR<<1, GYRO_XOUT_H_REG, 1, GyroArr, 6, 100);
	GyroRW[0] = (GyroArr[0]<<8)|GyroArr[1];
	GyroRW[1] = (GyroArr[2]<<8)|GyroArr[3];
	GyroRW[2] = (GyroArr[4]<<8)|GyroArr[5];
	//Gyro Raw Data
	rawDef->x = GyroRW[0];
	rawDef->y = -GyroRW[1];
	rawDef->z = -GyroRW[2];	
}

int16_t MPU6050_Get_Temp(void){
	uint8_t TemArr[2];
	int16_t t;
	HAL_I2C_Mem_Read (&hi2c1, MPU_ADDR<<1, TEMP_OUT_H_REG, 1, TemArr, 2, 100);
	t = (TemArr[0]<<8)|TemArr[1];
	return(t);
}

void HMC5883L_Config(void)
{	
	I2C_Write8(HMC5883L_ADDRESS, HMC5883L_Reg_A, 0x78);
	I2C_Write8(HMC5883L_ADDRESS, HMC5883L_Reg_B, 0x40);
	I2C_Write8(HMC5883L_ADDRESS, HMC5883L_Mode,  0x00);
	
//	I2C_Write8(MPU_ADDR, 0x37, 0x00);
//	I2C_Write8(MPU_ADDR, USER_CNT_REG, 0x20);
	
// configure the MPU6050 to automatically read the magnetometer
//	I2C_Write8(MPU_ADDR, 0x25, HMC5883L_ADDRESS | 0x80);
//	I2C_Write8(MPU_ADDR, 0x26, 0x03);
//	I2C_Write8(MPU_ADDR, 0x27, 6 | 0x80);
//	I2C_Write8(MPU_ADDR, 0x67, 1);
}

void HMC5883L_Get_Mag_RawData(RawData_Def *rawDef)
{
	uint8_t MagArr[6] = {0};
	//Magnetometer Raw Data
	HAL_I2C_Mem_Read (&hi2c1, HMC5883L_ADDRESS<<1, HMC5883L_X_MSB, 1, MagArr, 6, 100);
	rawDef->x = ((MagArr[0]<<8)|MagArr[1]);
	rawDef->z = ((MagArr[2]<<8)|MagArr[3]);
	rawDef->y = -((MagArr[4]<<8)|MagArr[5]);	
}

void MS5611_Init(void) {
	// Reset
	MS5611WriteByte(0x1E);
	HAL_Delay(100);

	// Read calibration data
	C[1] = MS5611ReadShort(0xA2);
	C[2] = MS5611ReadShort(0xA4);
	C[3] = MS5611ReadShort(0xA6);
	C[4] = MS5611ReadShort(0xA8);
	C[5] = MS5611ReadShort(0xAA);
	C[6] = MS5611ReadShort(0xAC);
}
