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
#include "mpu6050.h"
			

int main(void)
{
	mpu6050_data_t mpu6050_raw;
	debug_init();
	pwm_in_init();
	pwm_out_init();
    mpu6050_init();

    char out[100];

	for(;;)
	{
		debug_write("\033[2J");
		debug_write("\033[H");
		/* Read all data from sensor 1 */
		mpu6050_readall(&mpu6050_raw);
		bzero(out,sizeof(out));
		/* Format data */
		sprintf(out, "AX:%d - AY:%d - AZ:%d  |  GX:%d - GY:%d - GZ:%d  T:%3.4f\n",mpu6050_raw.AccX,
																				  mpu6050_raw.AccY,
																				  mpu6050_raw.AccZ,
																				  mpu6050_raw.GyroX,
																				  mpu6050_raw.GyroY,
																				  mpu6050_raw.GyroZ,
																				  mpu6050_raw.Temperature);

		debug_write(out);
		bzero(out,sizeof(out));
		sprintf(out,"\rthrottle: %d - roll: %d - pitch: %d - yaw: %d - aux0: %d - aux1: %d  \n", pwm_in_get_channel(THR_INDEX), pwm_in_get_channel(ROLL_INDEX), pwm_in_get_channel(PITCH_INDEX), pwm_in_get_channel(YAW_INDEX), pwm_in_get_channel(AUX0_INDEX), pwm_in_get_channel(AUX1_INDEX));
		pwm_out_set_period(0,pwm_in_get_channel(THR_INDEX));
		debug_write(out);
		Delay(500000);
	}
}
