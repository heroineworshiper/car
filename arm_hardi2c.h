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



#ifndef ARM_HARDI2C_H
#define ARM_HARDI2C_H


#include <stdint.h>
#include "stm32f4xx_i2c.h"

// doesn't work because I2C_FLAG_BUSY is polled
//#define I2C_INTERRUPTS

typedef struct 
{
	void (*state)(void *);
	
	I2C_TypeDef *regs;
	uint32_t dev_address;
	uint32_t reg_address;
    int pin_combo;
// Value for single byte reads & writes
	uint32_t value;
// burst output
	unsigned char burst[16];
// bytes to read in burst
	int bytes;
	int bytes_read;

	uint32_t want_event;
	int got_event;
// copy of SR1
	uint32_t status1;
	int timeout;
    int error;
} i2c_t;



#define i2c_ready(i2c) ((i2c)->state == i2c_idle)

#ifndef I2C_INTERRUPTS
#define handle_i2c(i2c) \
{ \
	(i2c)->state(i2c); \
}
#else
#define handle_i2c(i2c) \
{ \
}
#endif



void init_hardi2c(i2c_t *i2c, 
    I2C_TypeDef *regs, 
    int pin_combo);
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




#endif


