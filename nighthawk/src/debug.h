/*
 * debug.h
 *
 *  Created on: 6 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef DEBUG_H_
#define DEBUG_H_
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"

void Delay ( __IO uint32_t nCount );
void debug_init ( void );
void debug_write ( volatile char *info );

#endif /* DEBUG_H_ */
