/*
 * i2c.c
 *
 *  Created on: 17 de nov de 2015
 *      Author: ox_jodm_xo
 */

#include "i2c.h"

void i2c_init ( void )
{
		I2C_InitTypeDef I2C_InitStruct;
		GPIO_InitTypeDef GPIO_InitStruct;

		/* Enable clocks */
		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOB , ENABLE );
		RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1 , ENABLE );

		/* Configure Pins*/
		GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_25MHz;
		GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
		GPIO_Init( GPIOB , &GPIO_InitStruct );
		GPIO_PinAFConfig( GPIOB , GPIO_PinSource8 , GPIO_AF_I2C1 );
		GPIO_PinAFConfig( GPIOB , GPIO_PinSource9 , GPIO_AF_I2C1 );

		/* Configure i2c */
		I2C_InitStruct.I2C_ClockSpeed = 400000; // 400 kHz
		I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
		I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
		I2C_InitStruct.I2C_OwnAddress1 = 0x00;
		I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;
		I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;

		/* Disable I2C controller*/
		I2C_Cmd( I2C1 , DISABLE );

		/* Initialize I2C */
		I2C_Init( I2C1 , &I2C_InitStruct );

		/* Enable I2C controller*/
		I2C_Cmd( I2C1 , ENABLE );
}

int8_t i2c_start ( uint8_t addr , uint8_t direction , uint8_t ack )
{
		uint16_t timeout;
		/* Generate I2C start condition */
		I2C_GenerateSTART( I2C1 , ENABLE );

		/* Wait */
		timeout = I2C_TIMEOUT;
		while ( !I2C_CheckEvent( I2C1 , I2C_SR1_SB ) )
				if ( --timeout == 0x00 )
						return -1;

		/* Enable ack*/
		if ( ack )
				I2C_AcknowledgeConfig( I2C1 , ENABLE );

		/* Send request start */
		I2C_Send7bitAddress( I2C1 , addr , direction );

		/* Wait */
		if ( direction == I2C_Direction_Transmitter )
		{
				timeout = I2C_TIMEOUT;
				while ( !I2C_CheckEvent( I2C1 , I2C_SR1_ADDR ) )
						if ( --timeout == 0x00 )
								return -1;
		}
		else // I2C_Direction_Receiver
		{
				timeout = I2C_TIMEOUT;
				while ( !I2C_CheckEvent( I2C1 , I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED ) )
						if ( --timeout == 0x00 )
								return -1;
		}

		/* Clear ADDR flag by reading SR2 */
		I2C1->SR2;

		/* Success */
		return 0;
}

int8_t i2c_stop ( void )
{
		uint16_t timeout;
		/* Wait */
		timeout = I2C_TIMEOUT;
		while ( ( ( !I2C_CheckEvent( I2C1 , I2C_SR1_TXE ) ) || ( !I2C_CheckEvent( I2C1 , I2C_SR1_BTF ) ) ) )
				if ( --timeout == 0x00 )
						return -1;

		/* Generate stop condition */
		I2C_GenerateSTOP( I2C1 , ENABLE );

		/* Success */
		return 0;
}

uint8_t i2c_is_device_alive ( uint8_t addr )
{
		uint8_t is_alive = 0;
		/* Check if device send an acknowledge */
		if ( !i2c_start( addr , I2C_Direction_Transmitter , ENABLE ) )
				is_alive = 1;

		/* Generate stop condition */
		i2c_stop();

		/* Return if device is alive */
		return is_alive;
}

void i2c_write_byte ( uint8_t data )
{
		uint16_t timeout;
		/* Wait */
		timeout = I2C_TIMEOUT;
		while ( !I2C_CheckEvent( I2C1 , I2C_SR1_TXE ) && timeout )
				timeout--;

		/* Send data */
		I2C_SendData( I2C1 , data );
}

uint8_t i2c_read_byte_ack ( void )
{
		uint16_t timeout;
		uint8_t data;

		/* Enable ACK */
		I2C_AcknowledgeConfig( I2C1 , ENABLE );

		/* Wait */
		timeout = I2C_TIMEOUT;
		while ( !I2C_CheckEvent( I2C1 , I2C_EVENT_MASTER_BYTE_RECEIVED ) )
				if ( --timeout == 0x00 )
						return -1;

		/* Read data */
		data = I2C_ReceiveData( I2C1 );

		/* Return data */
		return data;
}

uint8_t i2c_read_byte_nack ( void )
{
		uint16_t timeout;
		uint8_t data;

		/* Disable ACK */
		I2C_AcknowledgeConfig( I2C1 , DISABLE );

		/* Generate stop condition */
		I2C_GenerateSTOP( I2C1 , ENABLE );

		/* Wait */
		timeout = I2C_TIMEOUT;
		while ( !I2C_CheckEvent( I2C1 , I2C_EVENT_MASTER_BYTE_RECEIVED ) )
				if ( --timeout == 0x00 )
						return -1;

		/* Read data */
		data = I2C_ReceiveData( I2C1 );

		/* Return data */
		return data;
}

void i2c_write_reg ( uint8_t addr , uint8_t reg , uint8_t data )
{
		i2c_start( addr , I2C_Direction_Transmitter , ENABLE );
		i2c_write_byte( reg );
		i2c_write_byte( data );
		i2c_stop();
}

uint8_t i2c_read_reg ( uint8_t addr , uint8_t reg )
{
		i2c_start( addr , I2C_Direction_Transmitter , ENABLE );
		i2c_write_byte( reg );
		i2c_stop();
		i2c_start( addr , I2C_Direction_Receiver , DISABLE );
		return i2c_read_byte_nack();
}

void i2c_read_buffer ( uint8_t addr , uint8_t reg , uint8_t* buffer , uint16_t size )
{
		uint16_t index;
		i2c_start( addr , I2C_Direction_Transmitter , ENABLE );
		i2c_write_byte( reg );
		i2c_stop();
		i2c_start( addr , I2C_Direction_Receiver , ENABLE );
		for ( index = 0; index < size ; index++ )
				if ( index == ( size - 1 ) )
						buffer[index] = i2c_read_byte_nack();
				else
						buffer[index] = i2c_read_byte_ack();
}
