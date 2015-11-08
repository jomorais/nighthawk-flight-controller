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
#include "pwm_out.h"
#include "debug.h"
			

int main(void)
{
	debug_init();
	pwm_in_init();
	pwm_out_init();
	char out[100];
	for(;;)
	{
		Delay(100000);
		bzero(out,sizeof(out));
		sprintf(out,"throttle: %d - roll: %d - pitch: %d - yaw: %d - aux0: %d - aux1: %d         \r", pwm_in_get_channel(THR_INDEX), pwm_in_get_channel(ROLL_INDEX), pwm_in_get_channel(PITCH_INDEX), pwm_in_get_channel(YAW_INDEX), pwm_in_get_channel(AUX0_INDEX), pwm_in_get_channel(AUX1_INDEX));
		pwm_out_set_period(0,pwm_in_get_channel(THR_INDEX));
		debug_write(out);
	}
}
