/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "pwm_in.h"
#include "pwm_out.h"
#include "debug.h"
#include "orientation.h"

int main ( void )
{

		utils_init();
		debug_init();
		pwm_in_init();
		pwm_out_init();
		orientation_init();

		char out[100];
		float dt = 0;
		uint32_t t = _micros();
		euler_angles_t e;
	  uint32_t dt_c = 20000;
		for ( ; ; )
		{

				t = _micros();
				/* Read all data from sensor 1 */
				orientation_update( 20.0 );

				e =  get_euler_angles();
				bzero( out , sizeof ( out ) );
				/* Format data */
				//int n = sprintf( out , "{\"p\":%f,\"r\":%f,\"y\":%f,\"dt\":%ld}\n" , e.pitch, e.roll, e.yaw, dt);
				sprintf( out, "{\"x\":%f,\"y\":%f,\"z\":%f,\"dt\":%f}\n" , angles.ANGLE_X, angles.ANGLE_Y, angles.ANGLE_Z , dt / 1000);
				//sprintf( out, "gx:%f, gy:%f, gz:%f, ax:%f, ay:%f, az:%f\n" , sensor_data.GX , sensor_data.GY , sensor_data.GZ , sensor_data.AX,sensor_data.AY,sensor_data.AZ);

				debug_write( out );
				//bzero( out , sizeof ( out ) );
				//sprintf( out , "\rthrottle: %d - roll: %d - pitch: %d - yaw: %d - aux0: %d - aux1: %d  \n" , pwm_in_get_channel( THR_INDEX ) , pwm_in_get_channel( ROLL_INDEX ) , pwm_in_get_channel( PITCH_INDEX ) , pwm_in_get_channel( YAW_INDEX ) , pwm_in_get_channel( AUX0_INDEX ) , pwm_in_get_channel( AUX1_INDEX ) );
				//pwm_out_set_period( 0 , pwm_in_get_channel( THR_INDEX ) );
				//debug_write( out );
				while((_micros() - t) < dt_c);
				//debug_write( "\033[2J" );
				//debug_write( "\033[H" );

		}
}
