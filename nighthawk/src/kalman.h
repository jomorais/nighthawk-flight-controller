/*
 * kalman.h
 *
 *  Created on: 30/11/2015
 *      Author: ox_jodm_xo
 */

#ifndef KALMAN_H_
#define KALMAN_H_
#include "utils.h"
#include "attributes.h"

#define ACC_KALMAN_P 1.05728
#define ACC_KALMAN_Q 0.125
#define ACC_KALMAN_R 10.0
#define ACC_KALMAN_K 0.10572799

#define GYRO_KALMAN_P 0.46972573
#define GYRO_KALMAN_Q 0.0625
#define GYRO_KALMAN_R 4.0
#define GYRO_KALMAN_K 0.11743143

kalman_state acc_kalman_state[AXIS];
kalman_state gyro_kalman_state[AXIS];


void kalman_init ( void );
void kalman_update ( void );

#endif /* KALMAN_H_ */
