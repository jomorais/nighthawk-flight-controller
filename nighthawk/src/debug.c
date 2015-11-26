/*
 * debug.c
 *
 *  Created on: 6 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "debug.h"

void Delay ( __IO uint32_t nCount )
{
		while ( nCount-- )
				;
}

void debug_init ( void )
{
		GPIO_InitTypeDef GPIO_InitStruct;
		USART_InitTypeDef USART_InitStruct;

		RCC_APB2PeriphClockCmd( RCC_APB2Periph_USART1 , ENABLE );
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );

		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init( GPIOB , &GPIO_InitStruct );

		GPIO_PinAFConfig( GPIOB , GPIO_PinSource6 , GPIO_AF_USART1 );
		GPIO_PinAFConfig( GPIOB , GPIO_PinSource7 , GPIO_AF_USART1 );

		USART_InitStruct.USART_BaudRate = 115200;				// the baudrate is set to the value we passed into this init function
		USART_InitStruct.USART_WordLength = USART_WordLength_8b;				// we want the data frame size to be 8 bits (standard)
		USART_InitStruct.USART_StopBits = USART_StopBits_1;		// we want 1 stop bit (standard)
		USART_InitStruct.USART_Parity = USART_Parity_No;		// we don't want a parity bit (standard)
		USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
		USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
		USART_Init( USART1 , &USART_InitStruct );

		USART_Cmd( USART1 , ENABLE );
}

void debug_write ( volatile char *info )
{
		while ( *info )
		{
				// wait until data register is empty
				while ( ! ( USART1->SR & 0x00000040 ) )
						;
				USART_SendData( USART1 , *info );
				info++;
		}
}
