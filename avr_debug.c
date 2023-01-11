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

// debugging for the AVR
// UART TX is used for the H bridge, so we're left with 9600 baud bit banging





void (*volatile uart_state)();
uint8_t current_bit;
uint8_t uart_data;

#define UART_PERIOD (-6400) // 2400
//#define UART_PERIOD (-1600) // 9600
//#define UART_PERIOD (-800) // 19200

// only 1024 bytes of SRAM on the ATMega8
#define UART_SIZE 128
uint8_t uart_buffer[UART_SIZE];
int uart_used = 0;
volatile uint8_t have_uart_in = 0;
volatile uint8_t uart_in = 0;

void print_text(const char *string)
{

	int i;
	for(i = 0; uart_used < UART_SIZE && string[i] != 0; i++)
	{
//bitClear(TX_PORT, TX_PIN);
		uart_buffer[uart_used++] = string[i];
	}
}

const char hex_table[] = 
{
	'0', '1', '2', '3', '4', '5', '6', '7', 
	'8', '9', 'a', 'b', 'c', 'd', 'e', 'f'
};


void print_bin(uint8_t number)
{
	char buffer[10];
	int i;
	for(i = 0; i < 8; i++)
	{
		if(number & 0x80)
		{
			buffer[i] = '1';
		}
		else
		{
			buffer[i] = '0';
		}
		number <<= 1;
	}
	buffer[8] = ' ';
	buffer[9] = 0;
	print_text(buffer);
}

void print_hex(int number)
{
	char buffer[8];
	uint8_t i;
	uint8_t force = 0;
	char *dst = buffer;
	
	for(i = 0; i < 4; i++)
	{
		uint8_t code = (number >> 12) & 0xf;
		
		if(code > 0 || force || i == 3)
		{
			force = 1;
			*dst++ = hex_table[code];
		}
		
		number <<= 4;
	}

	*dst++ = ' ';
	*dst = 0;
	print_text(buffer);
}

void print_number(int number)
{
	char string[8];
	char *ptr = string;
	if(number < 0)
	{
		*ptr++ = '-';
		number = -number;
	}

	if(number >= 10000) *ptr++ = '0' + (number / 10000);
	if(number >= 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number >= 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number >= 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

void print_number_unsigned(uint16_t number)
{
	char string[8];
	char *ptr = string;
	if(number >= 10000) *ptr++ = '0' + (number / 10000);
	if(number >= 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number >= 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number >= 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

// From Arduino
/* Delay for the given number of microseconds.  Assumes a 1, 8, 12, 16, 20 or 24 MHz clock. */
void delayMicroseconds(unsigned int us)
{
	// call = 4 cycles + 2 to 4 cycles to init us(2 for constant delay, 4 for variable)

	// calling avrlib's delay_us() function with low values (e.g. 1 or
	// 2 microseconds) gives delays longer than desired.
	//delay_us(us);
#if F_CPU >= 24000000L
	// for the 24 MHz clock for the aventurous ones, trying to overclock

	// zero delay fix
	if (!us) return; //  = 3 cycles, (4 when true)

	// the following loop takes a 1/6 of a microsecond (4 cycles)
	// per iteration, so execute it six times for each microsecond of
	// delay requested.
	us *= 6; // x6 us, = 7 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 22 (24) cycles above, remove 5, (5*4=20)
	// us is at least 6 so we can substract 5
	us -= 5; //=2 cycles

#elif F_CPU >= 20000000L
	// for the 20 MHz clock on rare Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 18 (20) cycles, which is 1us
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop" "\n\t"
		"nop" "\n\t"
		"nop"); //just waiting 4 cycles
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes a 1/5 of a microsecond (4 cycles)
	// per iteration, so execute it five times for each microsecond of
	// delay requested.
	us = (us << 2) + us; // x5 us, = 7 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 26 (28) cycles above, remove 7, (7*4=28)
	// us is at least 10 so we can substract 7
	us -= 7; // 2 cycles

#elif F_CPU >= 16000000L
	// for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/4 of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2; // x4 us, = 4 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 19 (21) cycles above, remove 5, (5*4=20)
	// us is at least 8 so we can substract 5
	us -= 5; // = 2 cycles,

#elif F_CPU >= 12000000L
	// for the 12 MHz clock if somebody is working with USB

	// for a 1 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 1.5us
	if (us <= 1) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/3 of a microsecond (4 cycles)
	// per iteration, so execute it three times for each microsecond of
	// delay requested.
	us = (us << 1) + us; // x3 us, = 5 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 20 (22) cycles above, remove 5, (5*4=20)
	// us is at least 6 so we can substract 5
	us -= 5; //2 cycles

#elif F_CPU >= 8000000L
	// for the 8 MHz internal clock

	// for a 1 and 2 microsecond delay, simply return.  the overhead
	// of the function call takes 14 (16) cycles, which is 2us
	if (us <= 2) return; //  = 3 cycles, (4 when true)

	// the following loop takes 1/2 of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1; //x2 us, = 2 cycles

	// account for the time taken in the preceeding commands.
	// we just burned 17 (19) cycles above, remove 4, (4*4=16)
	// us is at least 6 so we can substract 4
	us -= 4; // = 2 cycles

#else
	// for the 1 MHz internal clock (default settings for common Atmega microcontrollers)

	// the overhead of the function calls is 14 (16) cycles
	if (us <= 16) return; //= 3 cycles, (4 when true)
	if (us <= 25) return; //= 3 cycles, (4 when true), (must be at least 25 if we want to substract 22)

	// compensate for the time taken by the preceeding and next commands (about 22 cycles)
	us -= 22; // = 2 cycles
	// the following loop takes 4 microseconds (4 cycles)
	// per iteration, so execute it us/4 times
	// us is at least 4, divided by 4 gives us 1 (no zero delay bug)
	us >>= 2; // us div 4, = 4 cycles
	

#endif

	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);

// hack for the current compiler & chip
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");
	__asm__ __volatile__ ("nop");

	// return = 4 cycles
}


static void uart_delay()
{
	delayMicroseconds(104);
}

static void uart_delay2()
{
	delayMicroseconds(104 / 3);
}


void stop_bit_tx()
{
	bitSet(TX_PORT, TX_PIN);
// clear the start bit interrupt
	bitSet(GIFR, INTF0);
// disable the data interrupt
	bitClear(TIMSK, TOIE1);

#ifdef DO_RECEIVE
// enable start bit interrupt
	bitSet(GICR, INT0);
#endif

	uart_state = 0;
}

void data_bit_tx()
{
	if(uart_data & 0x1)
	{
		bitSet(TX_PORT, TX_PIN);
	}
	else
	{
		bitClear(TX_PORT, TX_PIN);
	}
	uart_data >>= 1;
	current_bit++;
	if(current_bit >= 8)
	{
		uart_state = stop_bit_tx;
	}
	TCNT1H = UART_PERIOD >> 8;
	TCNT1L = UART_PERIOD & 0xff;
}

void start_bit_tx()
{
// send start bit
	bitClear(TX_PORT, TX_PIN);
	current_bit = 0;
	TCNT1H = UART_PERIOD >> 8;
	TCNT1L = UART_PERIOD & 0xff;
	uart_state = data_bit_tx;
}


// send RS232 byte
void send_byte(unsigned char x)
{
// disable interrupts
//	cli();
	bitSet(TX_PORT, TX_PIN);
	
	uart_data = x;
// disable start bit interrupt
	bitClear(GICR, INT0);
	uart_state = start_bit_tx;
	TCNT1H = UART_PERIOD >> 8;
	TCNT1L = UART_PERIOD & 0xff;
// clear the data bit interrupt
	bitSet(TIFR, TOV1);
// enable data bit clock & interrupt
// no prescaler
	TCCR1B = 0x1;
	bitSet(TIMSK, TOIE1);


	while(uart_state != 0)
	{
	}



#if 0
// start bit
	bitClear(TX_PORT, TX_PIN);
	uart_delay();
// data bits
	uint8_t temp = x;
// important to use int for the timing
	uint8_t i;
	for(i = 0; i < 8; i++)
	{
		if(temp & 0x1)
		{
			bitSet(TX_PORT, TX_PIN);
		}
		else
		{
			bitClear(TX_PORT, TX_PIN);
		}
		uart_delay();
		temp >>= 1;
	}
	bitSet(TX_PORT, TX_PIN);

// need a stop bit
	uart_delay();
#endif


// enable interrupts
//	sei();
}


// handle input
// void handle_serial()
// {
// // got a character
// 	if(bitIsClear(RX_INPORT, RX_PIN))
// 	{
// // very important to wait for the middle of the start bit
// 		uart_delay2();
// 
// 		uint8_t i;
// 		for(i = 0; i < 8; i++)
// 		{
// 			uart_in >>= 1;
// 			uart_delay();
// 			if(bitIsSet(RX_INPORT, RX_PIN))
// 			{
// 				uart_in |= 0x80;
// 			}
// 		}
// 
// // wait for the end of the last bit
// 		uart_delay();
// 
// 		have_uart_in = 1;
// 	}	
// }

// handle output
void flush_serial()
{
	uint8_t i;
	for(i = 0; i < uart_used; i++)
	{
		send_byte(uart_buffer[i]);
	}

	uart_used = 0; 
}



// detect data bits using timer 2 overflows
ISR(TIMER1_OVF_vect)
{
	uart_state();
}



#ifdef DO_RECEIVE




// wait for end of last bit
void stop_bit()
{
//bitToggle(TX_PORT, TX_PIN); // DEBUG
// clear the start bit interrupt
	bitSet(GIFR, INTF0);
// disable the data interrupt
	bitClear(TIMSK, TOIE1);
// enable start bit interrupt
	bitSet(GICR, INT0);

	uart_in = uart_data;
	have_uart_in = 1;
	uart_state = 0;
}

void data_bits()
{
//bitToggle(TX_PORT, TX_PIN); // DEBUG
	uart_data >>= 1;
	if(bitIsSet(RX_INPORT, RX_PIN))
	{
		uart_data |= 0x80;
	}
	current_bit++;
	TCNT1H = (UART_PERIOD >> 8);
	TCNT1L = (UART_PERIOD & 0xff);

	if(current_bit >= 8)
	{
		uart_state = stop_bit;
	}
}

void start_bit()
{
//bitToggle(TX_PORT, TX_PIN); // DEBUG
// wait for 1st bit
	current_bit = 0;
	uart_state = data_bits;
	uart_data = 0;
	TCNT1H = (UART_PERIOD >> 8);
	TCNT1L = (UART_PERIOD & 0xff);
}

// detect start bit using INT0 transition
ISR(INT0_vect)
{
//bitToggle(TX_PORT, TX_PIN); // DEBUG
// disable start bit interrupt
	bitClear(GICR, INT0);

// wait 1/2 a clockcycle
	uart_state = start_bit;
	TCNT1H = (UART_PERIOD / 2) >> 8;
	TCNT1L = ((UART_PERIOD / 2) & 0xff);
// clear the data bit interrupt
	bitSet(TIFR, TOV1);

// enable data bit clock & interrupt
// no prescaler
	TCCR1B = 0x1;
	bitSet(TIMSK, TOIE1);
}



#endif // DO_RECEIVE

void init_serial()
{
// transmit pin
	bitSet(TX_DDR, TX_PIN);
	bitSet(TX_PORT, TX_PIN);

#ifdef DO_RECEIVE
// receive pin with pullup.  PWM routine needs to keep its PORT bit high
	bitClear(RX_DDR, RX_PIN);
	bitSet(RX_PORT, RX_PIN);

// falling edge interrupt for start bit
	bitSet(MCUCR, ISC01);
	bitClear(MCUCR, ISC00);
// enable interrupt for the start bit
	bitSet(GICR, INT0);
#endif // DO_RECEIVE
}







