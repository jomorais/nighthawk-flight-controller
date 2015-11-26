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
		quaternion.q0 = 1.0;
		quaternion.q1 = 0.0;
		quaternion.q2 = 0.0;
		quaternion.q3 = 0.0;
}

void orientation_calc_euler_angles ( quaternion_t *q , euler_angles_t *e )
{
		e->roll = atan2f( 2 * ( q->q0 * q->q1 + q->q2 * q->q3 ) , 1 - 2 * ( q->q1 * q->q1 + q->q2 * q->q2 ) ) * 57.3;
		e->pitch = asinf( 2 * ( q->q0 * q->q2 - q->q3 * q->q1 ) ) * 57.3;
		e->yaw = atan2f( 2 * ( q->q0 * q->q3 + q->q1 * q->q2 ) , 1 - 2 * ( q->q2 * q->q2 + q->q3 * q->q3 ) ) * 57.3;
}

quaternion_t get_quaternion ( void )
{
		return quaternion;
}

euler_angles_t get_euler_angles ( void )
{
		return euler_angles;
}

void orientation_update ( float delta )
{
		sensors_update( delta );

		//MadgwickAHRSupdate(sensor_data.GX * DEG_TO_RAD, sensor_data.GY * DEG_TO_RAD, sensor_data.GZ * DEG_TO_RAD, sensor_data.AX, sensor_data.AY, sensor_data.AZ,sensor_data.MX,sensor_data.MY,sensor_data.MZ, ((float)delta / 1000000) / 2);

	  //euler_angles.roll = atanf(sensor_data.AY / sqrt(sensor_data.AX * sensor_data.AX + sensor_data.AZ * sensor_data.AZ)) * RAD_TO_DEG;
		//euler_angles.pitch = atan2f(-sensor_data.AX , sensor_data.AZ) * RAD_TO_DEG;
		//euler_angles.yaw = sensor_data.MX;

		//MadgwickAHRSupdateIMU ( sensor_data.GX * DEG_TO_RAD, sensor_data.GY * DEG_TO_RAD, sensor_data.GZ * DEG_TO_RAD, sensor_data.AX , sensor_data.AY , sensor_data.AZ, ((float)delta / 1000000) );
}

