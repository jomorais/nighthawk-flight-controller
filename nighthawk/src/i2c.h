/*
 * i2c.h
 *
 *  Created on: 17 de nov de 2015
 *      Author: ox_jodm_xo
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f4xx.h"

#define I2C_TIMEOUT       2200

void i2c_init(void);
int8_t i2c_start(uint8_t addr, uint8_t direction, uint8_t ack);
int8_t i2c_stop(void);
void i2c_write_byte(uint8_t data);
uint8_t i2c_read_byte_ack(void);
uint8_t i2c_read_byte_nack(void);
uint8_t i2c_is_device_alive(uint8_t addr);
void i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data);
uint8_t i2c_read_reg(uint8_t addr, uint8_t reg);
void i2c_read_buffer(uint8_t addr, uint8_t reg, uint8_t* buffer, uint16_t size);


#endif /* I2C_H_ */
