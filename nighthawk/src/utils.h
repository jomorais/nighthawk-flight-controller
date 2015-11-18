/*
 * utils.h
 *
 *  Created on: 14 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef UTILS_H_
#define UTILS_H_

#include "stm32f4xx.h"

/* return min value */
#define _min(X,Y) ((X) < (Y) ? : (X) : (Y))

/* return max value */
#define _max(X,Y) ((X) > (Y) ? : (X) : (Y))

/* return absolute value */
#define _abs(X) ((X) < (0) ? : -1*(X) : (X))

/* return a valid value (V) between MIN and MAX */
#define _constrain(V,MIN,MAX) ((V) < (MIN) ? (MIN) : ((V)>(MAX) ? (MAX) : (V)))

#endif /* UTILS_H_ */
