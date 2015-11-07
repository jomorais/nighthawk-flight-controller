/*
 * pwm_in.c
 *
 *  Created on: 6 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "pwm_in.h"

input_channels input[6];
TIM_ICInitTypeDef TIM_ICInitStruct;

void pwm_in_gpio_init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz ;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

}

void pwm_in_timer_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_TimeBaseStructInit(&TIM_TimeBaseStruct);
	TIM_TimeBaseStruct.TIM_Period = 0x0000FFFF;
	TIM_TimeBaseStruct.TIM_Prescaler = (SystemCoreClock / 1000000 / 2) - 1; // 1MHz - 1uS
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

	TIM_TimeBaseStruct.TIM_Prescaler = (SystemCoreClock / 1000000 / 2) - 1; // 1MHz - 1uS
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

	TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStruct.TIM_ICFilter = 0;
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	TIM_ICInit(TIM3,&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	TIM_ICInit(TIM3,&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
	TIM_ICInit(TIM2,&TIM_ICInitStruct);

	/* TIM enable counter */
	TIM_Cmd(TIM2, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void pwm_in_interrupts_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the CC Interrupt Request */
	TIM_ITConfig(TIM2, TIM_IT_CC1 | TIM_IT_CC2 | TIM_IT_CC3 | TIM_IT_CC4, ENABLE);
	/* Enable the CC Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
}

void pwm_in_init(void)
{
	pwm_in_gpio_init();
	pwm_in_timer_init();
	pwm_in_interrupts_init();

}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_CC1)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1);
		if(input[THR_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[THR_INDEX].rising_value = TIM_GetCapture1(TIM2);
			input[THR_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[THR_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);

		}
		else
		{
			input[THR_INDEX].falling_value = TIM_GetCapture1(TIM2);
			if (input[THR_INDEX].falling_value < input[THR_INDEX].rising_value)
				input[THR_INDEX].capture = (0xFFFF - input[THR_INDEX].rising_value) + input[THR_INDEX].falling_value;
			else
				input[THR_INDEX].capture = input[THR_INDEX].falling_value - input[THR_INDEX].rising_value;

			input[THR_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[THR_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC2)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC2);
		if(input[ROLL_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[ROLL_INDEX].rising_value = TIM_GetCapture2(TIM2);
			input[ROLL_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[ROLL_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);

		}
		else
		{
			input[ROLL_INDEX].falling_value = TIM_GetCapture2(TIM2);
			if (input[ROLL_INDEX].falling_value < input[ROLL_INDEX].rising_value)
				input[ROLL_INDEX].capture = (0xFFFF - input[ROLL_INDEX].rising_value) + input[ROLL_INDEX].falling_value;
			else
				input[ROLL_INDEX].capture = input[ROLL_INDEX].falling_value - input[ROLL_INDEX].rising_value;

			input[ROLL_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[ROLL_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC3)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
		if(input[PITCH_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[PITCH_INDEX].rising_value = TIM_GetCapture3(TIM2);
			input[PITCH_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[PITCH_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);

		}
		else
		{
			input[PITCH_INDEX].falling_value = TIM_GetCapture3(TIM2);
			if (input[PITCH_INDEX].falling_value < input[PITCH_INDEX].rising_value)
				input[PITCH_INDEX].capture = (0xFFFF - input[PITCH_INDEX].rising_value) + input[PITCH_INDEX].falling_value;
			else
				input[PITCH_INDEX].capture = input[PITCH_INDEX].falling_value - input[PITCH_INDEX].rising_value;

			input[PITCH_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[PITCH_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_3;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);
		}
	}

	if (TIM_GetITStatus(TIM2, TIM_IT_CC4)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
		if(input[YAW_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[YAW_INDEX].rising_value = TIM_GetCapture4(TIM2);
			input[YAW_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[YAW_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);

		}
		else
		{
			input[YAW_INDEX].falling_value = TIM_GetCapture4(TIM2);
			if (input[YAW_INDEX].falling_value < input[YAW_INDEX].rising_value)
				input[YAW_INDEX].capture = (0xFFFF - input[YAW_INDEX].rising_value) + input[YAW_INDEX].falling_value;
			else
				input[YAW_INDEX].capture = input[YAW_INDEX].falling_value - input[YAW_INDEX].rising_value;

			input[YAW_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[YAW_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_4;
			TIM_ICInit(TIM2,&TIM_ICInitStruct);
		}
	}
}

void TIM3_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM3, TIM_IT_CC1)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
		if(input[AUX0_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[AUX0_INDEX].rising_value = TIM_GetCapture1(TIM3);
			input[AUX0_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[AUX0_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
			TIM_ICInit(TIM3,&TIM_ICInitStruct);

		}
		else
		{
			input[AUX0_INDEX].falling_value = TIM_GetCapture1(TIM3);
			if (input[AUX0_INDEX].falling_value < input[AUX0_INDEX].rising_value)
				input[AUX0_INDEX].capture = (0xFFFF - input[AUX0_INDEX].rising_value) + input[AUX0_INDEX].falling_value;
			else
				input[AUX0_INDEX].capture = input[AUX0_INDEX].falling_value - input[AUX0_INDEX].rising_value;

			input[AUX0_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[AUX0_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_1;
			TIM_ICInit(TIM3,&TIM_ICInitStruct);
		}
	}

	if (TIM_GetITStatus(TIM3, TIM_IT_CC2)) {
		/* Clear pending bit */
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		if(input[AUX1_INDEX].polarity == TIM_ICPolarity_Rising)
		{
			input[AUX1_INDEX].rising_value = TIM_GetCapture2(TIM3);
			input[AUX1_INDEX].polarity = TIM_ICPolarity_Falling;
			TIM_ICInitStruct.TIM_ICPolarity = input[AUX1_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
			TIM_ICInit(TIM3,&TIM_ICInitStruct);

		}
		else
		{
			input[AUX1_INDEX].falling_value = TIM_GetCapture2(TIM3);
			if (input[AUX1_INDEX].falling_value < input[AUX1_INDEX].rising_value)
				input[AUX1_INDEX].capture = (0xFFFF - input[AUX1_INDEX].rising_value) + input[AUX1_INDEX].falling_value;
			else
				input[AUX1_INDEX].capture = input[AUX1_INDEX].falling_value - input[AUX1_INDEX].rising_value;

			input[AUX1_INDEX].polarity = TIM_ICPolarity_Rising;
			TIM_ICInitStruct.TIM_ICPolarity = input[AUX1_INDEX].polarity;
			TIM_ICInitStruct.TIM_Channel = TIM_Channel_2;
			TIM_ICInit(TIM3,&TIM_ICInitStruct);
		}
	}
}

uint16_t pwm_in_get_channel(uint8_t channel)
{
	return input[channel].capture;
}
