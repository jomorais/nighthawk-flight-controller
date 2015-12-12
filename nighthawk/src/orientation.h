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
#include "MadgwickAHRS.h"

#define sampleFreq	60.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions



void orientation_init ( void );
void orientation_update ( float delta );

#endif /* ORIENTATION_H_ */
