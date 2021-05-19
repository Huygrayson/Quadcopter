#ifndef OTHERSENSORS_ADD
#define OTHERSENSORS_ADD

//HMC5883L
#define HMC5883L_ADDRESS 0x1E		//I2C 8-bit Address for HMC5883L
#define HMC5883L_Reg_A 0x00
#define HMC5883L_Reg_B 0x01
#define HMC5883L_Mode  0x02
#define HMC5883L_X_MSB 0x03
#define HMC5883L_X_LSB 0x04
#define HMC5883L_Z_MSB 0x05
#define HMC5883L_Z_LSB 0x06
#define HMC5883L_Y_MSB 0x07
#define HMC5883L_Y_LSB 0x08

//MS5611
#define MS5611_ADDRESS 0x77
#define MS5611_OSR	4 // 0..4
//OSR 0 - Resolution 0.065 mbar, Conversion time 0.60 ms
//OSR 1 - Resolution 0.042 mbar, Conversion time 1.17 ms
//OSR 2 - Resolution 0.027 mbar, Conversion time 2.28 ms
//OSR 3 - Resolution 0.018 mbar, Conversion time 4.54 ms
//OSR 4 - Resolution 0.012 mbar, Conversion time 9.04 ms

#endif /* OTHERSENSORS_ADD */
