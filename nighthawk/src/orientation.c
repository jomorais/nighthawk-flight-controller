/*
 * orientation.c
 *
 *  Created on: 21/11/2015
 *      Author: ox_jodm_xo
 */

#include "orientation.h"

void orientation_init ( void )
{
		sensors_init();
}

void orientation_calc_euler_angles_360 ( void )
{
		float m11 , m12 , m21 , m31 , m32;
		float gx = 0 , gy = 0 , gz = 0;
		float temp_angles[AXIS];

		// estimated gravity direction
		gx = 2 * ( q1 * q3 - q0 * q2 );
		gy = 2 * ( q0 * q1 + q2 * q3 );
		gz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		m11 = 2. * ( q1 * q2 + q0 * q3 );
		m12 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
		m21 = -2. * ( q1 * q3 - q0 * q2 );
		m31 = 2. * ( q2 * q3 + q0 * q1 );
		m32 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

		temp_angles[YAW] = -atan2( m11 , m12 ) * 57.2957795;
		temp_angles[PITCH] = -asin( m21 ) * 57.2957795;
		temp_angles[ROLL] = -atan2( m31 , m32 ) * 57.2957795;

		if ( gx >= 0 && gz < 0 )
				temp_angles[PITCH] = 180. - temp_angles[PITCH];
		else
				if ( gx < 0 && gz < 0 )
						temp_angles[PITCH] = 180. - temp_angles[PITCH];
				else
						if ( gx < 0 && gz >= 0 )
								temp_angles[PITCH] = 360. + temp_angles[PITCH];
		if ( temp_angles[YAW] < 0 )
				temp_angles[YAW] = 360. + temp_angles[YAW];
		if ( temp_angles[ROLL] < 0 )
				temp_angles[ROLL] = 360. + temp_angles[ROLL];

		temp_angles[YAW] = 360 - temp_angles[YAW];
		angles[YAW] = temp_angles[YAW];
		angles[PITCH] = temp_angles[PITCH];
		angles[ROLL] = temp_angles[ROLL];

}

void orientation_update ( float delta )
{
		sensors_update( delta );
		//MadgwickAHRSupdateIMU( gyro_smooth[X] * DEG_TO_RAD , gyro_smooth[Y] * DEG_TO_RAD , gyro_smooth[Z] * DEG_TO_RAD , acc_smooth[X] , acc_smooth[Y] , acc_smooth[Z] , ( (float) delta / 1000 ) );

}

