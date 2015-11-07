/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "pwm_in.h"
#include "debug.h"
			

int main(void)
{

	char out[100];
	bzero(out,sizeof(out));
	debug_init();
	pwm_in_init();
	for(;;)
	{
		Delay(1000000);
		sprintf(out,"throttle: %d - roll: %d - pitch: %d - yaw: %d           \r", pwm_in_get_channel(THR_INDEX), pwm_in_get_channel(1), pwm_in_get_channel(2), pwm_in_get_channel(3));
		debug_write(out);
	}
}
