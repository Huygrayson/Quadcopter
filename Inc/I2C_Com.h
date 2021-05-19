#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

#ifndef I2C_COM
#define I2C_COM

//1. Raw data typedef
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}RawData_Def;

//2. Scaled data typedef
typedef struct
{
	float x;
	float y;
	float z;
}ScaledData_Def;

typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

//Function Prototype
//1- i2c Handler 
void MPU6050_Init(I2C_HandleTypeDef *I2Chnd);
//2- i2c Read
void I2C_Read(uint8_t DEV, uint8_t ADDR, uint8_t *i2cBuf, uint8_t NofData);
//3- i2c Write 8 Bit
void I2C_Write8(uint8_t DEV, uint8_t ADDR, uint8_t data);
uint16_t MS5611ReadShort(uint8_t address);
uint32_t MS5611ReadLong(uint8_t address);
void MS5611WriteByte(uint8_t data);

#endif /* I2C_COM */

