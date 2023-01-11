/*
 * 3 phase motor driver from an ESC
 *
 * Copyright (C) 2021 Adam Williams <broadcast at earthling dot net>
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


#ifndef MOTOR_H
#define MOTOR_H



#ifdef __AVR__


#define F_CPU 16000000L

#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

#endif // __AVR__

#ifdef __x86_64__

#include  <stdint.h>

#endif // __x86_64__

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(reg, bit) ((reg) ^= (1 << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))
#define bitIsSet(value, bit) ((value) & (1UL << (bit)))
#define bitIsClear(value, bit) (!((value) & (1UL << (bit))))


#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))


// pins for the FETs
#define P_FET1	5
#define P_FET2	7
#define P_FET3	4
#define N_FET1	0
#define N_FET2	1
#define N_FET3	3

// use the PWM input pin for TX if no receive
#define TX_PIN 2
#define TX_PORT PORTD
#define TX_DDR DDRD



// the table played back by the interrupt handler
typedef struct
{
// members have to be volatile to be access by an interrupt handler
// time until the next interrupt
	volatile uint8_t time;
// value of the PORT register
	volatile uint8_t value;
} pwm_table_t;

// temporary table used for calculating the pwm table
typedef struct
{
// time since overflow
	int16_t time;
// Ps to turn off
	uint8_t p_mask;
// Ns to turn on
	uint8_t n_mask;
} event_table_t;


// need 9uS of deadband from P to N
//#define DEADBAND 104  // prescaler 1
//#define DEADBAND 12 // prescaler 2 with 18A Hobbyking
#define DEADBAND 128 // prescaler 2 with 35A Hobbyking

#define DEADBAND2 0 // minimum delay from N to P saves 20mA on 35A

// tweek this to get the right Hz
// need multiple padding events or higher power if period is too high
//#define PERIOD 582  // maximum before overflow prescaler 1 16khz
#define PERIOD 216  // prescaler 2 8khz
//#define PERIOD 108  // prescaler 2 16khz
//#define PERIOD 280  // prescaler 2 6khz


#define MAX_POWER ((int16_t)PERIOD)

#define MIN_POWER 1


void delayMicroseconds(unsigned int us);




#endif













