/*
 * mpu6050.h
 *
 *  Created on: 29/11/2015
 *      Author: ox_jodm_xo
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include "utils.h"
#include "i2c.h"
#include "attributes.h"

#define MPU6050_ADDR			          		0xD0
#define MPU6050_I_AM				        		0x68
#define MPU6050_WHO_AM_I								0x75
#define MPU6050_CONFIG									0x1A
#define MPU6050_GYRO_CONFIG							0x1B
#define MPU6050_ACCEL_CONFIG						0x1C
#define MPU6050_ACCEL_XOUT_H						0x3B
#define MPU6050_GYRO_XOUT_H							0x43
#define MPU6050_PWR_MGMT_1							0x6B
#define MPU6050_PWR_MGMT_2							0x6C
#define MPU6050_SCALE_FACTOR						0.030518509f // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
#define MPU6050_GYRO_GAIN_FACTOR_250dps 7.629627369f // ( 250.0f * MPU6050_SCALE_FACTOR )
#define MPU6050_ACC_GAIN_FACTOR_4G      1.197546293f // ( 4.0f * GRAVITY * MPU6050_SCALE_FACTOR)

int8_t mpu6050_init ( void );
void mpu6050_calibration_proccess ( void );
void mpu6050_read_acc_gyro ( void );

#endif /* MPU6050_H_ */
