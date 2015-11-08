/*
 * pwm_out.c
 *
 *  Created on: 7 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "pwm_out.h"

out_channels_t out_channels[4];

void pwm_out_gpio_init(void)
{
	/* GPIOD clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	/* GPIOD Configuration: Pins 12, 13, 14 and 15 in output push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Connect TIM4 pins to AF2 */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

}

void pwm_out_timer_init(void)
{
	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	/* Time base configuration - SystemCoreClock = 168000000 for 168 MHz board */
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (((SystemCoreClock / 1000000) / 2) - 1); // Shooting for 1 MHz, (1us)
	TIM_TimeBaseStructure.TIM_Period = 20000 - 1; // 1 MHz / 20000 = 50 Hz (20ms)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* Enable TIM4 Preload register on ARR */
	TIM_ARRPreloadConfig(TIM4, ENABLE);

	/* TIM PWM1 Mode configuration: Channel */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1500; // Servo Top-Center
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	/* Output Compare PWM1 Mode configuration: Channel1 PD.12 */
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* Output Compare PWM1 Mode configuration: Channel2 PD.13 */
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* Output Compare PWM1 Mode configuration: Channel3 PD.14 */
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* Output Compare PWM1 Mode configuration: Channel4 PD.15 */
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	/* TIM4 enable counter */
	TIM_Cmd(TIM4, ENABLE);
}

void pwm_out_interrupts_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
}

void pwm_out_init(void)
{
	pwm_out_gpio_init();
	pwm_out_timer_init();
	pwm_out_interrupts_init();
	int channel;
	for(channel = 0; channel < 4; channel++)
		out_channels[channel].period = 990;
}

void pwm_out_set_period(uint8_t channel ,uint16_t period)
{
	out_channels[channel].period = period;
}

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		TIM_SetCompare1(TIM4, out_channels[0].period ); // PD.12
		TIM_SetCompare2(TIM4, out_channels[1].period ); // PD.13
		TIM_SetCompare3(TIM4, out_channels[2].period ); // PD.14
		TIM_SetCompare4(TIM4, out_channels[3].period ); // PD.15
    }
}
