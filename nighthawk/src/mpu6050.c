/*
 * mpu6050.c
 *
 *  Created on: 29/11/2015
 *      Author: ox_jodm_xo
 */

#include "mpu6050.h"

int8_t mpu6050_init ( void )
{
		/* check device */
		if ( !i2c_is_device_alive( MPU6050_ADDR ) )
				return -1;

		/* Check who I am */
		if ( i2c_read_reg( MPU6050_ADDR , MPU6050_WHO_AM_I ) != MPU6050_I_AM )
				return -1;

		/* module reset */
		i2c_write_reg( MPU6050_ADDR , MPU6050_PWR_MGMT_1 , 0x80 );
		_delay_ms( 100 );
		i2c_write_reg( MPU6050_ADDR , MPU6050_PWR_MGMT_1 , 0x00 );
		/* write gyro gain */
		i2c_write_reg( MPU6050_ADDR , MPU6050_GYRO_CONFIG , 0 );
		/* write acc gain */
		i2c_write_reg( MPU6050_ADDR , MPU6050_ACCEL_CONFIG , 1 << 3 );

		/* Success */
		return 0;
}

void mpu6050_calibration_proccess ( void )
{
		uint16_t sample = 0;
		float acc_offset_filtered[AXIS] , gyro_offset_filtered[AXIS];

		/* write filter config */
		i2c_write_reg( MPU6050_ADDR , MPU6050_CONFIG , 0 );

		/* discard first read */
		mpu6050_read_acc_gyro();
		mpu6050_read_acc_gyro();
		uint8_t axi;
		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				acc_offset_filtered[axi] = (float)acc_raw[axi];
				gyro_offset_filtered[axi] = (float)gyro_raw[axi];
		}

		for ( sample = 1; sample < 2000 ; sample++ )
		{
				mpu6050_read_acc_gyro();
				for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
				{
						acc_offset_filtered[axi] = (float)( acc_offset_filtered[axi] + acc_raw[axi] ) / 2;
						gyro_offset_filtered[axi] = (float)( gyro_offset_filtered[axi] + gyro_raw[axi] ) / 2;
				}
		}

		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				if ( axi == Z )
						acc_offset[axi] = acc_offset_filtered[axi] - (float)( GRAVITY * 1000.0f / MPU6050_ACC_GAIN_FACTOR_4G );
				else
						acc_offset[axi] = acc_offset_filtered[axi];

				gyro_offset[axi] = gyro_offset_filtered[axi];
		}

		/* set filter configuration */
		i2c_write_reg( MPU6050_ADDR , MPU6050_CONFIG , 2 );
}

void mpu6050_read_acc_gyro ( void )
{
		uint8_t axi , data[14];

		/* Read full raw data, 14bytes */
		i2c_read_buffer( MPU6050_ADDR , MPU6050_ACCEL_XOUT_H , data , sizeof ( data ) );

		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				/* get raw data */
				acc_raw[axi] = (int16_t) ( data[2 * axi] << 8 | data[2 * axi + 1] );
				gyro_raw[axi] = (int16_t) ( data[8 + 2 * axi] << 8 | data[8 + 2 * axi + 1] );

				/* calibrate raw data */
				acc_calibrated[axi] = (float) ( acc_raw[axi] - acc_offset[axi] ) * ( MPU6050_ACC_GAIN_FACTOR_4G / 1000 );
				gyro_calibrated[axi] = (float) ( gyro_raw[axi] - gyro_offset[axi] ) * ( MPU6050_GYRO_GAIN_FACTOR_250dps / 1000 );
		}
}
