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
#include "utils.h"
#include "debug.h"
#include "orientation.h"

int main ( void )
{
		utils_init();
		debug_init();
		pwm_in_init();
		pwm_out_init();
		_delay_ms(2000);
		orientation_init();

		char out[200];
		float dt = 0;
		uint32_t t = _micros();
	  uint32_t dt_c = 20000;
		for ( ; ; )
		{

				t = _micros();
				/* Read all data from sensor 1 */
				orientation_update( 20.0 );

				bzero( out , sizeof ( out ) );
				/* Format data */
				sprintf( out, "{\"x\":%f,\"y\":%f,\"z\":%f,\"xx\":%f,\"xy\":%f,\"xz\":%f}\n" , angles[PITCH], angles[ROLL], angles[YAW],acc_smooth[PITCH], acc_smooth[ROLL], acc_smooth[YAW]);

				debug_write( out );
				while((_micros() - t) < dt_c);

		}
}
