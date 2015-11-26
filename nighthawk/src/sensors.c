/*
 * mpu6050.c
 *
 *  Created on: 18 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "sensors.h"

int8_t mpu6050_init ( void );
void mpu6050_reset ( void );
void mpu6050_apply_calibration_correction ( void );
void mpu6050_calibration_proccess ( void );
void mag3110_init ( void );

int8_t sensors_init ( void )
{
		/* set all offset values to 0 */
		bzero( &calibration , sizeof ( calibration ) );

		/* init i2c */
		i2c_init();

		/* init mpu6050 sensor */
		mpu6050_init();

		/* init mag3110 sensor */
		mag3110_init();

		/* Success */
		return 0;
}

void mpu6050_config_int_pin_interrupt ( void )
{
		GPIO_InitTypeDef GPIO_InitStruct;
		EXTI_InitTypeDef EXTI_InitStruct;
		NVIC_InitTypeDef NVIC_InitStruct;

		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOD , ENABLE );
		RCC_APB2PeriphClockCmd( RCC_APB2Periph_SYSCFG , ENABLE );

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_Init( GPIOD , &GPIO_InitStruct );

		SYSCFG_EXTILineConfig( EXTI_PortSourceGPIOD , EXTI_PinSource0 );

		EXTI_InitStruct.EXTI_Line = EXTI_Line0;
		EXTI_InitStruct.EXTI_LineCmd = ENABLE;
		EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
		EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
		EXTI_Init( &EXTI_InitStruct );

		NVIC_InitStruct.NVIC_IRQChannel = EXTI0_IRQn;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0x00;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0x00;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init( &NVIC_InitStruct );
}

void mpu6050_reset ( void )
{
		// module reset
		i2c_write_reg( MPU6050_ADDR , MPU6050_RA_PWR_MGMT_1 , 0x80 );
		_delay_ms( 100 );
		i2c_write_reg( MPU6050_ADDR , MPU6050_RA_PWR_MGMT_1 , 0x00 );
}

void mpu6050_set_gains ( uint8_t gyro , uint8_t accel )
{
		uint8_t gyro_gain , accel_gain;

		switch ( gyro )
		{
				case 0 :
				{
						calibration.gyro_scale_factor = (float) 250 * 0.0305; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
						gyro_gain = 0b00000000;
				}
						break;
				case 1 :
				{
						calibration.gyro_scale_factor = 500 * 0.0305; // each data is of 16 bits that means, 500 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
						gyro_gain = 0b00001000;
				}
						break;
				case 2 :
				{
						calibration.gyro_scale_factor = 1000 * 0.0305; // each data is of 16 bits that means, 1000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
						gyro_gain = 0b00010000;
				}
						break;
				case 3 :
				{
						calibration.gyro_scale_factor = 2000 * 0.0305;  // each data is of 16 bits that means, 2000 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
						gyro_gain = 0b00011000;
				}
						break;
				default :
				{
						calibration.gyro_scale_factor = (float) 250 * 0.0305; // each data is of 16 bits that means, 250 is divided along 2^(15)-1 = 32767 so for milli degree/s 0.0305 = 1000/32767
						gyro_gain = 0b00000000;
				}
						break;
		}

		switch ( accel )
		{
				case 0 :
				{
						calibration.acc_scale_factor = (float) 2 * GRAVITY * 0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
						accel_gain = 0b00000000;
				}
						break;
				case 1 :
				{
						calibration.acc_scale_factor = (float) 4 * GRAVITY * 0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
						accel_gain = 0b00000000;
				}
						break;
				case 2 :
				{
						calibration.acc_scale_factor = (float) 8 * GRAVITY * 0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
						accel_gain = 0b00000000;
				}
						break;
				case 3 :
				{
						calibration.acc_scale_factor = (float) 16 * GRAVITY * 0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
						accel_gain = 0b00000000;
				}
						break;
				default :
				{
						calibration.acc_scale_factor = (float) 2 * GRAVITY * 0.0305; // each data is of 16 bits that means, 2g is divided along 2^(15)-1 = 32767 so for milli m/s^2 0.0305 = 1000/32767
						accel_gain = 0b00000000;
				}
						break;
		}

		// write gyro gain
		i2c_write_reg( MPU6050_ADDR , MPU6050_GYRO_CONFIG , gyro_gain );
		// write acc gain
		i2c_write_reg( MPU6050_ADDR , MPU6050_ACCEL_CONFIG , accel_gain );

}

void mpu6050_set_dlpf ( int band_width )
{
		/*
		 Digital Low Pass Filter (DLPF)
		 F2 F1 F0    Bandwidth [Hz]
		 0  0  0
		 0  0  1      184
		 0  1  0      94
		 0  1  1      44
		 1  0  0      21
		 1  0  1      10
		 1  1  0      5
		 */
		if ( band_width < 0 || band_width > 6 )
		{
				band_width = 0;
		}
		/* write filter config */
		i2c_write_reg( MPU6050_ADDR , MPU6050_CONFIG , band_width );
}

int8_t mpu6050_init ( void )
{

		/* check device */
		if ( !i2c_is_device_alive( MPU6050_ADDR ) )
				return -1;

		/* Check who I am */
		if ( i2c_read_reg( MPU6050_ADDR , MPU6050_WHO_AM_I ) != MPU6050_I_AM )
				return -1;

		/* module reset */
		mpu6050_reset();
		/* set gains and scale factor */
		mpu6050_set_gains( 0 , 0 );
		/* set filter configuration */
		mpu6050_set_dlpf( 0 );
		/* make offset calibration */
		mpu6050_calibration_proccess();
		/* set filter configuration */
		mpu6050_set_dlpf( 6 );

		/* Success */
		return 0;
}

void mpu6050_read_acc ( void )
{
		uint8_t data[6];

		/* Read accelerometer data */
		i2c_read_buffer( MPU6050_ADDR , MPU6050_ACCEL_XOUT_H , data , sizeof ( data ) );

		sensor_data_raw.AX = (int16_t) ( data[0] << 8 | data[1] );
		sensor_data_raw.AY = (int16_t) ( data[2] << 8 | data[3] );
		sensor_data_raw.AZ = (int16_t) ( data[4] << 8 | data[5] );
}

void mpu6050_read_gyro ( void )
{
		uint8_t data[6];

		/* Read gyroscope data */
		i2c_read_buffer( MPU6050_ADDR , MPU6050_GYRO_XOUT_H , data , sizeof ( data ) );

		sensor_data_raw.GX = (int16_t) ( data[0] << 8 | data[1] );
		sensor_data_raw.GY = (int16_t) ( data[2] << 8 | data[3] );
		sensor_data_raw.GZ = (int16_t) ( data[4] << 8 | data[5] );
}

void mpu6050_read_acc_gyro ( void )
{
		uint8_t data[14];

		/* Read full raw data, 14bytes */
		i2c_read_buffer( MPU6050_ADDR , MPU6050_ACCEL_XOUT_H , data , sizeof ( data ) );

		/* Format accelerometer data */
		sensor_data_raw.AX = (int16_t) ( data[0] << 8 | data[1] );
		sensor_data_raw.AY = (int16_t) ( data[2] << 8 | data[3] );
		sensor_data_raw.AZ = (int16_t) ( data[4] << 8 | data[5] );

		/* Format gyroscope data */
		sensor_data_raw.GX = (int16_t) ( data[8] << 8 | data[9] );
		sensor_data_raw.GY = (int16_t) ( data[10] << 8 | data[11] );
		sensor_data_raw.GZ = (int16_t) ( data[12] << 8 | data[13] );

		/* apply calibration and scale correction */
		mpu6050_apply_calibration_correction();

}

void mpu6050_apply_calibration_correction ( void )
{
		sensor_data.AX = (float) ( sensor_data_raw.AX - calibration.AX_offset ) * ( calibration.acc_scale_factor / 1000 );
		sensor_data.AY = (float) ( sensor_data_raw.AY - calibration.AY_offset ) * ( calibration.acc_scale_factor / 1000 );
		sensor_data.AZ = (float) ( sensor_data_raw.AZ - calibration.AZ_offset ) * ( calibration.acc_scale_factor / 1000 );

		sensor_data.GX = (float) ( sensor_data_raw.GX - calibration.GX_offset ) * ( calibration.gyro_scale_factor / 1000 );
		sensor_data.GY = (float) ( sensor_data_raw.GY - calibration.GY_offset ) * ( calibration.gyro_scale_factor / 1000 );
		sensor_data.GZ = (float) ( sensor_data_raw.GZ - calibration.GZ_offset ) * ( calibration.gyro_scale_factor / 1000 );
}

void mpu6050_calibration_proccess ( void )
{
		int sample = 0;
		int16_t ax , ay , az , gx , gy , gz;

		mpu6050_read_acc_gyro();
		mpu6050_read_acc_gyro();
		ax = sensor_data_raw.AX;
		ay = sensor_data_raw.AY;
		az = sensor_data_raw.AZ;
		gx = sensor_data_raw.GX;
		gy = sensor_data_raw.GY;
		gz = sensor_data_raw.GZ;
		for ( sample = 1; sample < 2000 ; sample++ )
		{
				mpu6050_read_acc_gyro();

				ax = ( ax + sensor_data_raw.AX ) / 2;
				ay = ( ay + sensor_data_raw.AY ) / 2;
				az = ( az + sensor_data_raw.AZ ) / 2;
				gx = ( gx + sensor_data_raw.GX ) / 2;
				gy = ( gy + sensor_data_raw.GY ) / 2;
				gz = ( gz + sensor_data_raw.GZ ) / 2;
		}
		calibration.AX_offset = ax;
		calibration.AY_offset = ay;
		calibration.AZ_offset = az - (float) GRAVITY * 1000 / calibration.acc_scale_factor;
		calibration.GX_offset = gx;
		calibration.GY_offset = gy;
		calibration.GZ_offset = gz;
}

void sensor_get_angles ( float delta )
{
		angles.ANGLE_GYRO_X = ( sensor_data.GX * ( (float) delta / 1000 ) + angles.ANGLE_X );
		angles.ANGLE_GYRO_Y = ( sensor_data.GY * ( (float) delta / 1000 ) + angles.ANGLE_Y );
		angles.ANGLE_GYRO_Z = ( sensor_data.GZ * ( (float) delta / 1000 ) + angles.ANGLE_Z );

		if (sensor_data.AZ > 0)
		{
				angles.ANGLE_ACC_X = atanf( sensor_data.AX / ( sqrt( sensor_data.AY * sensor_data.AY + sensor_data.AZ * sensor_data.AZ ) ) );
		}
		else
		{
				if (sensor_data.AZ < 0)
				{
						angles.ANGLE_ACC_X = M_PI - atanf( sensor_data.AX / ( sqrt( sensor_data.AY * sensor_data.AY + sensor_data.AZ * sensor_data.AZ ) ) );
				}
		}

		if (angles.ANGLE_ACC_X > 0 )
		{
				angles.ANGLE_ACC_X = 2 * M_PI - angles.ANGLE_ACC_X;
		}
		else
		{
				if (angles.ANGLE_ACC_X < 0)
				{
						angles.ANGLE_ACC_X = -angles.ANGLE_ACC_X;
				}
		}
		angles.ANGLE_ACC_X *= RAD_TO_DEG;

		/*
		if (sensor_data.AZ > 0)
				{
						angles.ANGLE_ACC_Y = atanf( sensor_data.AY / ( sqrt( sensor_data.AX * sensor_data.AX + sensor_data.AZ * sensor_data.AZ ) ) );
				}
				else
				{
						if (sensor_data.AZ < 0)
						{
								angles.ANGLE_ACC_Y = M_PI - atanf( sensor_data.AY / ( sqrt( sensor_data.AX * sensor_data.AX + sensor_data.AZ * sensor_data.AZ ) ) );
						}
				}

				if (angles.ANGLE_ACC_Y > 0 )
				{
						angles.ANGLE_ACC_Y = 2 * M_PI - angles.ANGLE_ACC_Y;
				}
				else
				{
						if (angles.ANGLE_ACC_Y < 0)
						{
								angles.ANGLE_ACC_Y = -angles.ANGLE_ACC_Y;
						}
				}
				angles.ANGLE_ACC_Y *= RAD_TO_DEG;
*/
		//angles.ANGLE_ACC_X = atanf( sensor_data.AX / ( sqrt( sensor_data.AY * sensor_data.AY + sensor_data.AZ * sensor_data.AZ ) ) ) * RAD_TO_DEG;
	  angles.ANGLE_ACC_Y = -atanf( sensor_data.AY / ( sqrt( sensor_data.AX * sensor_data.AX + sensor_data.AZ * sensor_data.AZ ) ) ) * RAD_TO_DEG;
		angles.ANGLE_ACC_Z = atanf( ( sqrt( sensor_data.AY * sensor_data.AY + sensor_data.AX * sensor_data.AX ) ) / sensor_data.AZ  ) * RAD_TO_DEG;

		angles.ANGLE_X = FILTER_GAIN * angles.ANGLE_GYRO_X + ( 1 - FILTER_GAIN ) * angles.ANGLE_ACC_X;
		angles.ANGLE_Y = FILTER_GAIN * angles.ANGLE_GYRO_Y + ( 1 - FILTER_GAIN ) * angles.ANGLE_ACC_Y;
		angles.ANGLE_Z = FILTER_GAIN * angles.ANGLE_GYRO_Z + ( 1 - FILTER_GAIN ) * angles.ANGLE_ACC_Z;
}

void mag3110_init ( void )
{
		_delay_us( 100 );
		i2c_write_reg( MAG_ADDRESS , MAG_CTRL_REG2 , 0x80 );  //Sensor Reset
		_delay_us( 100 );
		i2c_write_reg( MAG_ADDRESS , MAG_CTRL_REG1 , 0x11 ); // DR=20Hz / OSratio=64 / mode=Active
		_delay_us( 100 );
}

void mag3110_read_mag ( void )
{
		uint8_t data[6];

		/* Read full raw data, 6 bytes */
		i2c_read_buffer( MAG_ADDRESS , MAG_DATA_REGISTER , data , sizeof ( data ) );

		sensor_data_raw.MX = (int16_t) ( data[0] << 8 | data[1] );
		sensor_data_raw.MY = (int16_t) ( data[2] << 8 | data[3] );
		sensor_data_raw.MZ = (int16_t) ( data[4] << 8 | data[5] );
}

void mag3110_calibration_proccess ( void )
{
		// todo
}

void sensors_update ( float delta )
{
		mpu6050_read_acc_gyro();
		sensor_get_angles( delta );
}

void EXTI0_IRQHandler ( void )
{
		if ( EXTI_GetITStatus( EXTI_Line0 ) != RESET )
		{
				// todo
				/* Clear interrupt flag */
				EXTI_ClearITPendingBit( EXTI_Line0 );
		}
}
