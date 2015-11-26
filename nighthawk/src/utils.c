/*
 * utils.c
 *
 *  Created on: 14 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "utils.h"

void utils_init ( void )
{
		RCC_ClocksTypeDef RCC_Clocks;
		RCC_GetClocksFreq( &RCC_Clocks );
		/* considering 4 cycles to each while loop */
		_delay_multiplier = RCC_Clocks.HCLK_Frequency / ( 8 * 1000000 );

		TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
		NVIC_InitTypeDef NVIC_InitStructure;
		/* Enable the TIM5 gloabal Interrupt */
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

		/* TIM5 clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		/* Time base configuration */
		TIM_TimeBaseStruct.TIM_Period = 0xFFFFFFFF;
		TIM_TimeBaseStruct.TIM_Prescaler = ( SystemCoreClock / 1000000 / 2 ) - 1; // 1MHz - 1uS
		TIM_TimeBaseStruct.TIM_ClockDivision = 0;
		TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStruct);
		/* TIM IT enable */
		TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
		/* TIM2 enable counter */
		TIM_Cmd(TIM5, ENABLE);
}

void _delay_us ( uint32_t us )
{
		us = us * _delay_multiplier - 10;
		while ( us-- );
}

void _delay_ms ( uint32_t ms )
{
		ms = 1000 * ms * _delay_multiplier - 10;
		while ( ms-- );
}

uint32_t _micros ( void )
{
		uint32_t micros  = TIM5->CNT;
		//TIM_SetCounter(TIM5,0x00000000);
		return micros;
}

float _millis ( void )
{
		return (float)_micros() / 1000.0;
}

void TIM5_IRQHandler ( void )
{
		if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
		{
				TIM_ClearITPendingBit( TIM5 , TIM_IT_Update);
		}
}

/*
float invSqrt ( float x )
{
		uint32_t i = 0x5F1F1412 - ( *(uint32_t*) &x >> 1 );
		float tmp = *(float*) &i;
		return tmp * ( 1.69000231f - 0.714158168f * x * tmp * tmp );
}
*/

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

