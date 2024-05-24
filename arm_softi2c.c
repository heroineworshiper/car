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


// Software I2C


#include "settings.h"
#include "linux.h"
#include "arm_softi2c.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "uart.h"











//#define OPTIMIZE_I2C





#define I2C_TIMEOUT_MAX 1000

#define I2C_WRITE(value) \
	i2c->write_byte = value; \
	i2c->counter = 0; \
	i2c->state1 = i2c_write1;

#define I2C_READ \
/* preload the NACK if a single byte or ACK if a burst */ \
    if(i2c->bytes_read >= i2c->bytes - 1) \
	{ \
        i2c->write_byte = (1 << 7); \
    } \
    else \
    { \
        i2c->write_byte = 0; \
    } \
	i2c->counter = 0; \
	i2c->state1 = i2c_read1; \
/* data must rise before clock to read the byte */ \
	SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin)

#define I2C_START \
{ \
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin) \
	SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin) \
	i2c->state1 = i2c_start2; \
}


#define I2C_STOP i2c->state1 = i2c_stop1;

// SDA high to low while SCL is high
static void i2c_start3(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	CLEAR_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin); 
	i2c->state1 = i2c->state3;
}

static void i2c_start2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	CLEAR_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin); 
	i2c->state1 = i2c_start3;
}

static void i2c_start1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_START
}


// SDA low to high while SCL is high
static void i2c_stop3(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin)
	i2c->state1 = i2c->state3;
}

static void i2c_stop2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin)
	i2c->state1 = i2c_stop3;
}

static void i2c_stop1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	CLEAR_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin)
	i2c->state1 = i2c_stop2;
}





static void i2c_read_bit2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// if clock is being held low, wait
	if(!PIN_IS_SET(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin) && i2c->timeout++ < I2C_TIMEOUT_MAX)
	{
		return;
	}

	if(i2c->timeout++ >= I2C_TIMEOUT_MAX)
	{
        i2c->error = 1;
		TRACE
		print_text("I2C timeout");
	}

// read bit
	i2c->value |= PIN_IS_SET(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
// lower clock
	CLEAR_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin);
// go to high level state
	i2c->state1 = i2c->state2;
}


static void i2c_read_bit1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// clock high
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin)
	i2c->timeout = 0;
	i2c->state1 = i2c_read_bit2;
}



static void i2c_write_bit3(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// go to high level state
	i2c->state1 = i2c->state2;
}

static void i2c_write_bit2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// clock low
	CLEAR_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin);
	i2c->state1 = i2c_write_bit3;
}

#ifndef OPTIMIZE_I2C
static void i2c_write_bit1b(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;

// raise clock.  Must be after data pin
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin);
	i2c->state1 = i2c_write_bit2;
}

static void i2c_write_bit1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// most significant bit or ACK to data pin
	if(i2c->write_byte & 0x80)
	{
		SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
	}
	else
	{
		CLEAR_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
	}

	i2c->state1 = i2c_write_bit1b;
}


#else // !OPTIMIZE_I2C

static void i2c_write_bit1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// most significant bit or ACK to data pin
	if(i2c->write_byte & 0x80)
	{
		SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
	}
	else
	{
		CLEAR_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
	}

// raise clock.  Must be after data pin
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin);
	i2c->state1 = i2c_write_bit2;
}

#endif // OPTIMIZE_I2C







static void i2c_write1(void *ptr);

static void i2c_write2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// shift bits
	i2c->write_byte <<= 1;
// increase counter
	i2c->counter++;
	if(i2c->counter >= 8)
	{
// set next high level state when ACK is done
		i2c->state2 = i2c->state3;
// read ACK
		i2c_read_bit1(i2c);
	}
	else
	{
// write next bit
		i2c_write1(i2c);
	}
}

static void i2c_write1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
// set next high level state when bit is done
	i2c->state2 = i2c_write2;
// write next bit
	i2c_write_bit1(i2c);
}

static void i2c_read1(void *ptr);

static void i2c_read2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;

// increase counter
	i2c->counter++;
// skip if done
	if(i2c->counter >= 8)
	{
// set next high level state when bit is done
		i2c->state2 = i2c->state3;
// write ACK
		i2c_write_bit1(i2c);
	}
	else
	{
// read next bit
		i2c_read1(i2c);
	}
}

static void i2c_read1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;

// shift result
	i2c->value <<= 1;
// set next high level state when bit is done
	i2c->state2 = i2c_read2;
// read next bit
	i2c_read_bit1(i2c);
}






void i2c_idle(void *i2c)
{
}


void init_softi2c(i2c_t *i2c,
	int delay,
	void *data_gpio,
	void *clock_gpio,
	int data_pin,
	int clock_pin)
{
	bzero(i2c, sizeof(i2c_t));
	
	i2c->delay = delay;
	i2c->data_gpio = data_gpio;
	i2c->clock_gpio = clock_gpio;
	i2c->data_pin = data_pin;
	i2c->clock_pin = clock_pin;
	i2c->state1 = i2c->state2 = i2c->state3 = i2c_idle;
	GPIO_InitTypeDef  GPIO_InitStructure;

// Now breaks PWM
	GPIO_InitStructure.GPIO_Pin = i2c->clock_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(((GPIO_TypeDef*)i2c->clock_gpio), &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = i2c->data_pin;
	GPIO_Init(((GPIO_TypeDef*)i2c->data_gpio), &GPIO_InitStructure);



// raise pins
	SET_PIN(((GPIO_TypeDef*)i2c->clock_gpio), i2c->clock_pin);
	SET_PIN(((GPIO_TypeDef*)i2c->data_gpio), i2c->data_pin);
}









void i2c_write_device4(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_STOP
	i2c->state3 = i2c_idle;
}

void i2c_write_device3(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->value);
	i2c->state3 = i2c_write_device4;
}

void i2c_write_device2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->reg_address);
	i2c->state3 = i2c_write_device3;
}

void i2c_write_device1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->dev_address);
	i2c->state3 = i2c_write_device2;
}

void i2c_write_device(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
	unsigned char value)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = value;
	i2c->state3 = i2c_write_device1;
	I2C_START
}

void i2c_read_device6(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;

    i2c->burst[i2c->bytes_read++] = i2c->value;
    if(i2c->bytes_read >= i2c->bytes)
    {
	    I2C_STOP
	    i2c->state3 = i2c_idle;
    }
    else
// read next byte in a burst
    {
        i2c->value = 0;
	    I2C_READ
    }

/*
 * TRACE
 * print_hex(i2c->reg_address);
 * print_text("=");
 * print_hex(i2c->value);
 */
}

void i2c_read_device5(void *ptr)
{
// read byte, followed by NACK if a single byte to stop transmission
// or an ACK to start next byte if a burst
	i2c_t *i2c = (i2c_t*)ptr;
    
	I2C_READ
	i2c->state3 = i2c_read_device6;
}

void i2c_read_device4(void *ptr)
{
// device address
// bit 0 is 1 for read
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->dev_address | 0x1);
	i2c->state3 = i2c_read_device5;
}

void i2c_read_device3(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	i2c->state1 = i2c_start1;
	i2c->state3 = i2c_read_device4;
}

void i2c_read_device2(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->reg_address);
	i2c->state3 = i2c_read_device3;
}

void i2c_read_device1(void *ptr)
{
	i2c_t *i2c = (i2c_t*)ptr;
	I2C_WRITE(i2c->dev_address);
	i2c->state3 = i2c_read_device2;
}

void i2c_read_device(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = 0;
    i2c->bytes = 1;
  	i2c->bytes_read = 0;
	i2c->state3 = i2c_read_device1;
	I2C_START
}

void i2c_read_burst(i2c_t *i2c,
	unsigned char dev_address, 
	unsigned char reg_address,
    int bytes)
{
	i2c->dev_address = dev_address;
	i2c->reg_address = reg_address;
	i2c->value = 0;
	i2c->state3 = i2c_read_device1;

	i2c->bytes = bytes;
  	i2c->bytes_read = 0;
    int i;
	for(i = 0; i < bytes; i++)
		i2c->burst[i] = 0;


	I2C_START
}

void flush_i2c(i2c_t *i2c)
{
	while(!i2c_ready(i2c))
	{
		mdelay(1);
		handle_i2c_nodelay(i2c);
//		handle_uart();
        HANDLE_UART_OUT
/* Reload IWDG counter */
		IWDG->KR = KR_KEY_RELOAD;
	}
}








