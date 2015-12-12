/*
 * types.h
 *
 *  Created on: 29/11/2015
 *      Author: ox_jodm_xo
 */

#ifndef TYPES_H_
#define TYPES_H_

typedef struct
{
		float q; //process noise covariance
		float r; //measurement noise covariance
		float x; //value
		float p; //estimation error covariance
		float k; //kalman gain
} kalman_state;

#endif /* TYPES_H_ */
