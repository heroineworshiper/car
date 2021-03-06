/*
 * Leg test program
 *
 * Copyright (C) 2017 Adam Williams <broadcast at earthling dot net>
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
// compile with make leg
// program with make leg_isp
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



#include "leg.h"
#include <string.h> // memcpy



volatile uint8_t have_new_table = 0;
// entry in pwm_table we're playing back
pwm_table_t *pwm_ptr;

#define DO_RECEIVE
#include "avr_debug.c"
#include "pwm_routines.c"

// timer 2 overflow
ISR(TIMER2_OVF_vect)
{

// set the FET gates
	PORTD = pwm_ptr->value;
// set the next delay
	TCNT2 = pwm_ptr->time;


// next program line
	pwm_ptr++;

// new cycle begins
	if(pwm_ptr >= &pwm_table[TABLE_SIZE])
	{
		pwm_ptr = &pwm_table[0];

// copy new program in
		if(have_new_table)
		{
			memcpy(pwm_table, new_table, TABLE_SIZE * sizeof(pwm_table_t));
			have_new_table = 0;
		}
	}
}

void pwm_on()
{
// enable the overflow interrupt
	bitSet(TIMSK, TOIE2);
}

void pwm_off()
{
// disable the overflow interrupt & clear the FETs
	bitClear(TIMSK, TOIE2);
	PORTD = (1 << RX_PIN);
}

void print_menu()
{
	print_text("1 - turn\n");
	print_text("2 - jump\n");
	print_text("3 - hold\n");
	print_text("4 - print pwm\n");
	flush_serial();
}

void jump()
{
	int i, j;
	phase = 0;
	power = MAX_POWER / 2;
	print_text("jumping\n");
	flush_serial();
	pwm_on();



	for(i = 0; i < 100; i++)
	{
		delayMicroseconds(1000);
		uint8_t step = i / 8;
		if(step > 12) step = 12;
		if(step < 1) step = 1;
		phase += step;
		phase_to_pwm();
	}


	for(i = 0; i < 50; i++)
	{
		delayMicroseconds(1000);
	}

	pwm_off();
	print_text("done\n");
	flush_serial();
	print_menu();
}

void hold()
{
	int i;
	print_text("holding\n");
	flush_serial();
	phase = 0;
	power = MAX_POWER / 8;
	pwm_on();

	for(i = 0; i < 600; i++)
	{
		delayMicroseconds(1000);
		phase += 1;
		if(i == 599) power = MAX_POWER / 10;
		

		phase_to_pwm();

	}

	
	
// hold it
	for(i = 0; i < 1000; i++)
	{
		delayMicroseconds(1000);
	}

	pwm_off();
//	dump_pwm();
	print_text("done\n");
	flush_serial();
	print_menu();
}


void turn()
{
	print_text("press any key to stop\n");
	flush_serial();
	phase = 0;
	power = MAX_POWER / 16;
	pwm_on();
	phase_to_pwm();
	
	int i;
	while(1)
	{
		delayMicroseconds(1000);

		if(have_uart_in)
		{
			have_uart_in = 0;
			break;
		}

		phase += 4;
		phase_to_pwm();
	}
	
	pwm_off();
	print_text("done\n");
	flush_serial();
	print_menu();
}

void workaround()
{
}

int main()
{
// disable watchdog.  Disabled by default.
//	WDTCR = (1 << 4) | (1 << 3);
//	WDTCR = (1 << 3) | (1);
	

// clear the FETs.  Must clear pins as soon as it boots or current from
// the serial port will retain the SRAM values from when it was powered down &
// fry the MOSFETs.
	PORTD = 0;

// enable the FET outputs.  Makes no difference, because of pullups.
 	bitSet(DDRD, N_FET1);
 	bitSet(DDRD, N_FET2);
 	bitSet(DDRD, N_FET3);
 	bitSet(DDRD, P_FET1);
 	bitSet(DDRD, P_FET2);
 	bitSet(DDRD, P_FET3);



	

	init_serial();

// delay
	int i;
	for(i = 0; i < 1000; i++)
	{
		delayMicroseconds(1000);
	}




	power = MIN_POWER;
//	phase = 0xc0 + 85;
//	phase = 235;
	phase = 0;
	pwm_ptr = &pwm_table[0];
	pwm_off();




// set the PWM prescaler
//	TCCR2 = 0x1;
	TCCR2 = 0x2;
//	TCCR2 = 0x7;

// enable interrupts
	sei();
	

	print_text("****\nWelcome to the leg controller\n");
	flush_serial();


#if 0
	power = MIN_POWER;
	phase_to_pwm();
	dump_pwm();

	pwm_on();


	while(1)
	{
		for(i = 0; i < 1; i++)
		{
			delayMicroseconds(1000);
		}
		
		phase += 1;
		phase_to_pwm();

		if(have_uart_in)
		{
			have_uart_in = 0;
			
			send_byte(uart_in);
			switch(uart_in)
			{
				case '+':
					phase++;
					phase_to_pwm();
					dump_pwm();
					break;
				case '-':
					phase--;
					phase_to_pwm();
					dump_pwm();
					break;
			}
		}
	}
#endif // 0



// user input
	print_menu();
	while(1)
	{
		if(have_uart_in)
		{
			have_uart_in = 0;
			
			send_byte(uart_in);
			
			switch(uart_in)
			{
				case '1':
					turn();
					break;
			
				case '2':
					jump();
					break;

				case '3':
					hold();
					break;

				case '4':
					phase_to_pwm();
					dump_pwm();
					break;

// can't receive when PWM is on, so just take any character
//				default:
//					pwm_off();
//					break;
			}
			
//			print_hex(uart_in);
//			flush_serial();
//			jump();
		}
	}


#if 0
	while(1)
	{
		for(i = 0; i < 1; i++)
		{
			delayMicroseconds(1000);
		}
		
		bitSet(TX_PORT, TX_PIN); // DEBUG
		phase++;
		phase_to_pwm();
	}
#endif


}






















