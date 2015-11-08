/*
 * pwm_out.h
 *
 *  Created on: 7 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef PWM_OUT_H_
#define PWM_OUT_H_

#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "debug.h"

#define CHANNEL_OUT_0 0
#define CHANNEL_OUT_1 1
#define CHANNEL_OUT_2 2
#define CHANNEL_OUT_3 3

typedef struct
{
	uint16_t period;
}out_channels_t;

out_channels_t out_channels[4];

void pwm_out_init(void);
void pwm_out_set_period(uint8_t channel ,uint16_t period);


#endif /* PWM_OUT_H_ */
