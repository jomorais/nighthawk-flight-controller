/*
 * utils.h
 *
 *  Created on: 14 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stm32f4xx.h"
#include "types.h"
#include "config.h"
#include <string.h>
#include <math.h>

#define RAD_TO_DEG          57.295779513f // (180.0 / M_PI)
#define DEG_TO_RAD          0.017453293f  // (M_PI / 180.0)
#define GRAVITY             9.81f         // earth gravity

/* return min value */
#define _min(X,Y) ((X) < (Y) ? : (X) : (Y))

/* return max value */
#define _max(X,Y) ((X) > (Y) ? : (X) : (Y))

/* return absolute value */
#define _abs(X) ((X) < (0) ? : -1*(X) : (X))

/* return a valid value (V) between MIN and MAX */
#define _constrain(V,MIN,MAX) ((V) < (MIN) ? (MIN) : ((V)>(MAX) ? (MAX) : (V)))

/* return a invSqrt of (x) value */
float invSqrt ( float x );

/* multiplier to get a precise calc of delay */
uint32_t _delay_multiplier;

void utils_init ( void );

/* delay us (microsenconds) */
void _delay_us ( uint32_t us );

/* delay ms (milisenconds) */
void _delay_ms ( uint32_t ms );

uint32_t _micros ( void );

float _millis ( void );

#endif /* UTILS_H_ */
