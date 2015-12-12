/*
 * attributes.h
 *
 *  Created on: 29/11/2015
 *      Author: ox_jodm_xo
 */

#ifndef ATTRIBUTES_H_
#define ATTRIBUTES_H_

#include "utils.h"

#define AXIS 					3
#define FIRST_AXI 		0
#define X    					0
#define Y    					1
#define Z    					2
#define PITCH					X
#define ROLL 					Y
#define YAW  					Z

/* accelerometer raw data */
int16_t acc_raw[AXIS];
/* accelerometer offset calculated on calibration process */
int16_t acc_offset[AXIS];
/* accelerometer calibrated data */
float acc_calibrated[AXIS];
/* filtered accelerometer data */
float acc_smooth[AXIS];

/* gyroscope raw data */
int16_t gyro_raw[AXIS];
/* gyroscope offset calculated on calibration process */
int16_t gyro_offset[AXIS];
/* calibrated gyroscope data */
float gyro_calibrated[AXIS];
/* filtered gyroscope data */
float gyro_smooth[AXIS];

/* magnetometer raw data */
int16_t mag_raw[AXIS];
/* magnetometer offset calculated on calibration process */
int16_t mag_offset[AXIS];
/* calibrated magnetometer offset calculated on calibration process */
float mag_calibrated[AXIS];
/* filtered gyroscope data */
float mag_smooth[AXIS];

/* quaternion data */
float quaternion[4];

/* euler angles */
float angles[AXIS];
float euler[AXIS];
float euler180[AXIS];
float euler360[AXIS];

#endif /* ATTRIBUTES_H_ */
