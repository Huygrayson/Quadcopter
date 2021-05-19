#include "I2C_Com.h"
#include "OtherSensors_Add.h"

extern I2C_HandleTypeDef hi2c1;
//2- i2c Read
void I2C_Read(uint8_t DEV, uint8_t ADDR, uint8_t *i2cBif, uint8_t NofData)
{
	uint8_t i2cBuf[2] = {0};
	//Need to Shift address to make it proper to i2c operation
	uint8_t DEVADDR = (DEV<<1);
	i2cBuf[0] = ADDR;
	HAL_I2C_Master_Transmit(&hi2c1, DEVADDR, i2cBuf, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, DEVADDR, i2cBif, NofData, 100);
}

//3- i2c Write 8 Bit
void I2C_Write8(uint8_t DEV, uint8_t ADDR, uint8_t data)
{
	uint8_t i2cData[2];
	i2cData[0] = ADDR;
	i2cData[1] = data;
	uint8_t DEVADDR = (DEV<<1);
	HAL_I2C_Master_Transmit(&hi2c1, DEVADDR, i2cData, 2,100);
}

uint16_t MS5611ReadShort(uint8_t address) {

	uint8_t rxBuf[2] = {0};
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MS5611_ADDRESS << 1, &address, 1, 0x100);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MS5611_ADDRESS << 1, rxBuf, 2, 100);
	return (rxBuf[0] << 8) | rxBuf[1];
}


uint32_t MS5611ReadLong(uint8_t address) {

	uint8_t rxBuf[3] = {0};
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MS5611_ADDRESS << 1, &address, 1, 0x100);
	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)MS5611_ADDRESS << 1, rxBuf, 3, 100);
	return (rxBuf[0] << 16) | (rxBuf[1] << 8) | rxBuf[2];
}


void MS5611WriteByte(uint8_t data) {

	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)MS5611_ADDRESS << 1, &data, 1, 0x100);
}
