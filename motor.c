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


// Runs on the ATmega
// compile with make motor
// program with make motor_isp
// UART is 9600 baud

// pin functions
// 1 - field 3 N
// 2 - field 3 P
// 9 - field 1 P
// 11 - field 2 P
// 30 - field 1 N
// 31 - field 2 N
// 32 - debug RX
// 13 - debug TX



#include "motor.h"
#include <string.h> // memcpy

#define HZ 61
volatile uint32_t tick = 0;
volatile uint8_t got_tick = 0;
uint8_t timeout_counter = 0;
uint16_t counter = 0;

#include "avr_debug.c"
#include "pwm_routines.c"

// timer 0 overflow
ISR(TIMER0_OVF_vect)
{
    tick++;
    got_tick = 1;
}



int main()
{
// enable watchdog.  Disabled by default.
	WDTCR = 0b00011000;
// set prescaler
	WDTCR = 0b00001110;
	
// must be 1st
    init_pwm();

	

// delay
	int i;
	for(i = 0; i < 1000; i++)
	{
// reset watchdog
        asm("wdr");
		delayMicroseconds(1000);
	}



	init_serial();

// sys tick
    TCCR0 = 0b00000101;
    bitSet(TIMSK, TOIE0);

// enable interrupts
	sei();
	

	print_text("****\nWelcome to the motor controller\n");
	flush_serial();

// debug
phase = 0;
power = MAX_POWER / 2;
phase_to_pwm();
pwm_on();


	while(1)
	{
        asm("wdr");
        counter++;

// reset watchdog
        bitClear(TIMSK, TOIE0);
        if(got_tick && (tick % HZ) == 0)
        {
            got_tick = 0;
            bitSet(TIMSK, TOIE0);
            
            print_text("counter=");
            print_number(counter);
            counter = 0;
            print_text("\n");
            flush_serial();
        }
        else
        {
            bitSet(TIMSK, TOIE0);
        }
	}



}






















