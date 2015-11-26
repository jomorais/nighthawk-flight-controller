/*
 * pwm_in.h
 *
 *  Created on: 6 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef PWM_IN_H_
#define PWM_IN_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "debug.h"

#define THR_INDEX    0
#define ROLL_INDEX   1
#define PITCH_INDEX  2
#define YAW_INDEX    3
#define AUX0_INDEX   4
#define AUX1_INDEX   5

typedef struct
{
		uint16_t rising_value;
		uint16_t falling_value;
		uint16_t polarity;
		uint16_t capture;
} input_channels;

extern input_channels input[6];

void pwm_in_init ( void );
uint16_t pwm_in_get_channel ( uint8_t channel );

#endif /* PWM_IN_H_ */
