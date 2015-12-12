/*
 * kalman.c
 *
 *  Created on: 30/11/2015
 *      Author: ox_jodm_xo
 */
#include "kalman.h"

void kalman_init ( void )
{
		uint8_t axi;
		// init kalman terms
		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				acc_kalman_state[axi].q = ACC_KALMAN_Q;
				acc_kalman_state[axi].r = ACC_KALMAN_R;
				acc_kalman_state[axi].p = ACC_KALMAN_P;
				acc_kalman_state[axi].x = acc_calibrated[axi];

				gyro_kalman_state[axi].q = GYRO_KALMAN_Q;
				gyro_kalman_state[axi].r = GYRO_KALMAN_R;
				gyro_kalman_state[axi].p = GYRO_KALMAN_P;
				gyro_kalman_state[axi].x = gyro_calibrated[axi];
		}
}

void kalman_update ( void )
{
		uint8_t axi;
		for ( axi = FIRST_AXI; axi < AXIS ; axi++ )
		{
				//prediction update
				acc_kalman_state[axi].p = acc_kalman_state[axi].p + acc_kalman_state[axi].q;
				//measurement update
				acc_kalman_state[axi].k = acc_kalman_state[axi].p / ( acc_kalman_state[axi].p + acc_kalman_state[axi].r );
				acc_kalman_state[axi].x = acc_kalman_state[axi].x + acc_kalman_state[axi].k * ( acc_calibrated[axi] - acc_kalman_state[axi].x );
				acc_smooth[axi] = acc_kalman_state[axi].x;
				acc_kalman_state[axi].p = ( 1 - acc_kalman_state[axi].k ) * acc_kalman_state[axi].p;

				//prediction update
				gyro_kalman_state[axi].p = gyro_kalman_state[axi].p + gyro_kalman_state[axi].q;
				//measurement update
				gyro_kalman_state[axi].k = gyro_kalman_state[axi].p / ( gyro_kalman_state[axi].p + gyro_kalman_state[axi].r );
				gyro_kalman_state[axi].x = gyro_kalman_state[axi].x + gyro_kalman_state[axi].k * ( gyro_calibrated[axi] - gyro_kalman_state[axi].x );
				gyro_smooth[axi] = gyro_kalman_state[axi].x;
				gyro_kalman_state[axi].p = ( 1 - gyro_kalman_state[axi].k ) * gyro_kalman_state[axi].p;
		}
}
