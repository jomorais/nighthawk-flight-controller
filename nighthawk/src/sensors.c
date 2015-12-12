/*
 * mpu6050.c
 *
 *  Created on: 18 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "sensors.h"

int8_t sensors_init ( void )
{
		/* init i2c */
		i2c_init();

		/* init mpu6050 sensor */
		mpu6050_init();

		/* calibrate mpu6050 */
		mpu6050_calibration_proccess();

		/* init kalman filter */
		kalman_init();

		/* Success */
		return 0;
}

void sensor_calculate_angles ( float delta )
{
		float angle_gyro[AXIS] , angle_acc[AXIS];
		float *acc , *gyro;
		uint8_t axi;

		if ( KALMAN_FILTER )
		{
				acc = acc_smooth;
				gyro = gyro_smooth;
		}
		else
		{
				acc = acc_calibrated;
				gyro = gyro_calibrated;
		}

		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				angle_gyro[axi] = ( gyro[axi] * ( (float) delta / 1000 ) + angles[axi] );
		}

		if ( acc[Z] >= 0 )
		{
				angle_acc[X] = atan2f( acc[X] , ( sqrt( acc[Y] * acc[Y] + acc[Z] * acc[Z] ) ) ) * RAD_TO_DEG;
				angle_acc[Y] = atan2f( acc[Y] , ( sqrt( acc[X] * acc[X] + acc[Z] * acc[Z] ) ) ) * RAD_TO_DEG;
				angle_acc[Z] = -atan2f( ( sqrt( acc[Y] * acc[Y] + acc[X] * acc[X] ) ) , acc[Z] ) * RAD_TO_DEG;
		}
		else
		{
				angle_acc[X] = ( M_PI - atan2f( acc[X] , ( sqrt( acc[Y] * acc[Y] + acc[Z] * acc[Z] ) ) ) ) * RAD_TO_DEG;
				angle_acc[Y] = ( M_PI - atan2f( acc[Y] , ( sqrt( acc[X] * acc[X] + acc[Z] * acc[Z] ) ) ) ) * RAD_TO_DEG;
				angle_acc[Z] = ( M_PI + atan2f( ( sqrt( acc[Y] * acc[Y] + acc[X] * acc[X] ) ) , acc[Z] ) ) * RAD_TO_DEG;
		}

		angles[X] = FILTER_GAIN * angle_gyro[X] + ( 1 - FILTER_GAIN ) * angle_acc[X];
		angles[Y] = FILTER_GAIN * angle_gyro[Y] + ( 1 - FILTER_GAIN ) * angle_acc[Y];
		angles[Z] += gyro[Z] * ( (float) delta / 1000 );
}

void sensors_update ( float delta )
{
		mpu6050_read_acc_gyro();
		kalman_update();
		sensor_calculate_angles( delta );
}
