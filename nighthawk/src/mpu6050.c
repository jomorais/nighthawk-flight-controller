/*
 * mpu6050.c
 *
 *  Created on: 18 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "mpu6050.h"


int8_t mpu6050_init(void)
{
	uint8_t temp;
	/* init i2c */
	i2c_init();

	/* check device */
	if (i2c_is_device_alive(MPU6050_ADDR))
		return -1;

	/* Check who I am */
	if (i2c_read_reg(MPU6050_ADDR, MPU6050_WHO_AM_I) != MPU6050_I_AM)
		return -1;

	/* Wakeup MPU6050 */
	i2c_write_reg(MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x00);

	/* Config Accelerometer */
	temp = i2c_read_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)0x02 << 3;
	i2c_write_reg(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, temp);

	/* Config Gyroscope */
	temp = i2c_read_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG);
	temp = (temp & 0xE7) | (uint8_t)0x00 << 3;
	i2c_write_reg(MPU6050_ADDR, MPU6050_GYRO_CONFIG, temp);

	/* Success */
	return 0;
}

int8_t mpu6050_read_accelerometer(mpu6050_data_t* mpu6050_data)
{
	uint8_t data[6];

	/* Read accelerometer data */
	i2c_read_buffer(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, sizeof(data));

	mpu6050_data->AccX = (int16_t)(data[0] << 8 | data[1]);
	mpu6050_data->AccY = (int16_t)(data[2] << 8 | data[3]);
	mpu6050_data->AccZ = (int16_t)(data[4] << 8 | data[5]);

	/* Success */
	return 0;
}

int8_t mpu6050_read_gyro(mpu6050_data_t* mpu6050_data) {
	uint8_t data[6];

	/* Read gyroscope data */
	i2c_read_buffer(MPU6050_ADDR, MPU6050_GYRO_XOUT_H, data, sizeof(data));

	mpu6050_data->GyroX = (int16_t)(data[0] << 8 | data[1]);
	mpu6050_data->GyroY = (int16_t)(data[2] << 8 | data[3]);
	mpu6050_data->GyroZ = (int16_t)(data[4] << 8 | data[5]);

	/* Success */
	return 0;
}

int8_t mpu6050_read_temperature(mpu6050_data_t* mpu6050_data) {
	uint8_t data[2];
	int16_t temp;

	/* Read temperature */
	i2c_read_buffer(MPU6050_ADDR, MPU6050_TEMP_OUT_H, data, sizeof(data));

	temp = (data[0] << 8 | data[1]);
	mpu6050_data->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Success */
	return 0;
}


int8_t mpu6050_readall(mpu6050_data_t* mpu6050_data)
{
	uint8_t data[14];
	int16_t temp;

	/* Read full raw data, 14bytes */
	i2c_read_buffer(MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, sizeof(data));

	/* Format accelerometer data */
	mpu6050_data->AccX = (int16_t)(data[0] << 8 | data[1]);
	mpu6050_data->AccY = (int16_t)(data[2] << 8 | data[3]);
	mpu6050_data->AccZ = (int16_t)(data[4] << 8 | data[5]);

	/* Format temperature */
	temp = (data[6] << 8 | data[7]);
	mpu6050_data->Temperature = (float)((float)((int16_t)temp) / (float)340.0 + (float)36.53);

	/* Format gyroscope data */
	mpu6050_data->GyroX = (int16_t)(data[8] << 8 | data[9]);
	mpu6050_data->GyroY = (int16_t)(data[10] << 8 | data[11]);
	mpu6050_data->GyroZ = (int16_t)(data[12] << 8 | data[13]);

	/* Success */
	return 0;
}
