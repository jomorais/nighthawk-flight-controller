/*
 * orientation.h
 *
 *  Created on: 21/11/2015
 *      Author: ox_jodm_xo
 */

#ifndef ORIENTATION_H_
#define ORIENTATION_H_

#include "utils.h"
#include <math.h>
#include "sensors.h"

#define sampleFreq	60.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

typedef struct
{
				float q0;
				float q1;
				float q2;
				float q3;
}quaternion_t;

typedef struct
{
				float yaw;
				float pitch;
				float roll;
}euler_angles_t;


quaternion_t quaternion;
euler_angles_t euler_angles;

void orientation_init ( void );
void orientation_update ( float delta );
euler_angles_t get_euler_angles ( void );

#endif /* ORIENTATION_H_ */
