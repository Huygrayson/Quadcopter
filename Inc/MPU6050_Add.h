#ifndef MPU6050_ADD
#define MPU6050_ADD

#define WHO_AM_I_REG			0x75
#define MPU_ADDR					0x68
#define PWR_MGMT_1_REG		0x6B
#define CONFIG_REG				0x1A
#define GYRO_CONFIG_REG		0x1B
#define ACCEL_CONFIG_REG	0x1C
#define SMPLRT_DIV_REG		0x19
#define INT_STATUS_REG		0x3A
#define ACCEL_XOUT_H_REG	0x3B
#define TEMP_OUT_H_REG		0x41
#define GYRO_XOUT_H_REG		0x43
#define FIFO_EN_REG 			0x23
#define INT_ENABLE_REG 		0x38
#define I2CMACO_REG 			0x23
#define USER_CNT_REG			0x6A
#define FIFO_COUNTH_REG 	0x72
#define FIFO_R_W_REG 			0x74

#endif /* MPU6050_ADD */
