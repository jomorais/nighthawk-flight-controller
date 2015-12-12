/*
 * mpu6050.h
 *
 *  Created on: 18 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#include "i2c.h"
#include "utils.h"
#include "mpu6050.h"
#include "attributes.h"
#include "kalman.h"

#define  FILTER_GAIN 0.2

#define MAG_ADDRESS       0x1C
#define MAG_DATA_REGISTER 0x01
#define MAG_CTRL_REG1     0x10
#define MAG_CTRL_REG2     0x11
#define MAG3110_OFF_X_MSB 0x09
#define MAG_3110_RAW      0x20
#define MAG_3110_DR_STATUS 0x00
#define MAG_3110_ZYXDR     0x08


#define SENSOR_AXIS 3



int8_t sensors_init ( void );
void sensors_update ( float delta );

#endif /* SENSORS_H_ */
