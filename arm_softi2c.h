/*
 * STM32 Controller for direct drive truck
 * Copyright (C) 2012-2021 Adam Williams <broadcast at earthling dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 * 
 */

#ifndef ARM_SOFTI2C_H
#define ARM_SOFTI2C_H


// Software I2C
#include <stdint.h>

#define i2c_ready(i2c) ((i2c)->state1 == i2c_idle)
#define handle_i2c(i2c) \
{ \
	if(++(i2c)->delay_counter > (i2c)->delay) \
	{ \
		(i2c)->delay_counter = 0; \
 		(i2c)->state1(i2c); \
	} \
}

#define handle_i2c_nodelay(i2c) \
{ \
	(i2c)->state1(i2c); \
}

typedef struct 
{
	void (*state1)(void *);
	void (*state2)(void *);
	void (*state3)(void *);
	
	uint32_t dev_address;
	uint32_t reg_address;
// Value for single byte reads & writes
	uint32_t value;
// burst output
	unsigned char burst[16];
// bytes to read in burst
	int bytes;
	int bytes_read;

	uint32_t write_byte;
	uint32_t counter;
	int timeout;
	int delay_counter;
	void *data_gpio;
	void *clock_gpio;
	int data_pin;
	int clock_pin;
	int delay;
    int error;
} i2c_t;

void init_softi2c(i2c_t *i2c,
	int delay,
	void *data_gpio,
	void *clock_gpio,
	int data_pin,
	int clock_pin);
void i2c_write_device(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	unsigned char value);
void i2c_read_device(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address);
void i2c_read_burst(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	int bytes);
void i2c_idle(void *i2c);
void flush_i2c(i2c_t *i2c);


#endif

