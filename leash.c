/*
 * Smart leash
 * Copyright (C) 2022 Adam Williams <broadcast at earthling dot net>
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

// make leash_fuse
// make leash.hex


#define F_CPU 8000000L

#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

#define HZ 125
#define FRACTION 256
#define LED_ON PORTB &= ~(1 << PORTB5);
#define LED_OFF PORTB |= (1 << PORTB5);
#define LED_TOGGLE PORTB ^= (1 << PORTB5);

#define BAUD 115200L

#define POT_CENTER 512
// 45 deg values
#define POT_LEFT 906
#define POT_RIGHT 88

#define HALL_CENTER 512
#define HALL_THRESHOLD 256

#define ENCODER_START 0
#define HALL0_LOW 1
#define HALL1_LOW 2
#define HALL0_HIGH 3
#define HALL1_HIGH 4
int8_t encoder_state = ENCODER_START;
int8_t prev_encoder = ENCODER_START;
int16_t encoder_counts = 0;
uint8_t debug_count = 0;

uint16_t hall0_accum = 0;
uint16_t hall1_accum = 0;
uint16_t pot_accum = 0;
uint16_t hall0_value = 0;
uint16_t hall1_value = 0;
uint16_t pot_value = 0;
// angle * FRACTION
int16_t pot_angle = 0;
uint8_t hall_count = 0;
uint8_t pot_count = 0;
#define HALL_OVERSAMPLE 1
#define POT_OVERSAMPLE 64
uint8_t tick;


#define UART_SIZE 64
uint8_t uart_buffer[UART_SIZE];
uint8_t uart_used = 0;
uint8_t uart_write_ptr = 0;
uint8_t uart_read_ptr = 0;

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define CLAMP(x, y, z) ((x) = ((x) < (y) ? (y) : ((x) > (z) ? (z) : (x))))
#define MAX(x, y) ((x) > (y) ? (x) : (y))
#define MIN(x, y) ((x) < (y) ? (x) : (y))
#define ABS(x) ((x) < 0 ? (-(x)) : (x))



// send data to the UART buffer
void send_uart(uint8_t *text, uint8_t size)
{
	uint8_t i;
	for(i = 0; uart_used < UART_SIZE && i < size; i++)
	{
		uart_buffer[uart_write_ptr++] = text[i];
		uart_used++;
		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
	}
}

void print_text(char *text)
{
	uint8_t i;
	for(i = 0; uart_used < UART_SIZE && text[i] != 0; i++)
	{
		uart_buffer[uart_write_ptr++] = text[i];
		uart_used++;
		if(uart_write_ptr >= UART_SIZE) uart_write_ptr = 0;
	}
}

void print_number_nospace(int number)
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
	*ptr = 0;
	print_text(string);
}

void print_number(int number)
{
    print_number_nospace(number);
	print_text(" ");
}


void print_fixed(int16_t number)
{
    print_number_nospace(number / FRACTION);
	char string[8];
	if(number < 0) number = -number;
    uint8_t fraction = number % FRACTION;
    string[0] = '.';
    fraction = fraction * 100 / FRACTION;
    string[1] = '0' + (fraction / 10);
    string[2] = '0' + (fraction % 10);
    string[3] = ' ';
    string[4] = 0;
	print_text(string);
}

void handle_serial()
{
	if(uart_used) 
	{
	    if(bitRead(UCSR0A, UDRE0)) 
	    {
			bitSet(UCSR0A, UDRE0); 
			UDR0 = uart_buffer[uart_read_ptr++]; 
			if(uart_read_ptr >= UART_SIZE) uart_read_ptr = 0; 
			uart_used--; 
	    }
	}
}

void init_serial()
{
	uint16_t baud_setting = (F_CPU / 4 / BAUD - 1) / 2;
	UBRR0H = baud_setting >> 8;
	UBRR0L = baud_setting & 0xff;
	UCSR0A = (1 << U2X0);
	UCSR0C = (1 << UCSZ01) |
		(1 << UCSZ00);
	UCSR0B = (1 << RXCIE0) |
		(1 << TXEN0);
//         |
//		(1 << RXEN0);
}



void handle_encoder()
{
    if(hall1_value <= HALL_CENTER - HALL_THRESHOLD)
    {
        encoder_state = HALL1_LOW;
    }
    else
    if(hall1_value >= HALL_CENTER + HALL_THRESHOLD)
    {
        encoder_state = HALL1_HIGH;
    }
    else
    if(hall0_value <= HALL_CENTER - HALL_THRESHOLD)
    {
        encoder_state = HALL0_LOW;
    }
    else
    if(hall0_value >= HALL_CENTER + HALL_THRESHOLD)
    {
        encoder_state = HALL0_HIGH;
    }
    
    if(encoder_state != prev_encoder)
    {
        if(prev_encoder == HALL1_LOW && encoder_state == HALL0_LOW ||
            prev_encoder == HALL0_LOW && encoder_state == HALL1_HIGH ||
            prev_encoder == HALL1_HIGH && encoder_state == HALL0_HIGH ||
            prev_encoder == HALL0_HIGH && encoder_state == HALL1_LOW)
            encoder_counts++;
        else
        if(prev_encoder == HALL0_LOW && encoder_state == HALL1_LOW ||
            prev_encoder == HALL1_LOW && encoder_state == HALL0_HIGH ||
            prev_encoder == HALL0_HIGH && encoder_state == HALL1_HIGH ||
            prev_encoder == HALL1_HIGH && encoder_state == HALL0_LOW)
            encoder_counts--;

        prev_encoder = encoder_state;
    }
}

void main()
{
// disable watchdog
	WDTCSR = 0;
	init_serial();
	print_text("\n\nWelcome to smart leash\n");

// LED/DEBUG
    DDRB |= 1 << DDB5;
    LED_ON

// tick clock prescaler page 108
// CLKio is 32khz
    TCCR0B = 0b00000100;

#define ADMUX_BASE 0b01000000
    ADMUX = ADMUX_BASE;
    ADCSRA = 0b10000111;
    bitSet(ADCSRA, ADSC);
    
// enable interrupts
	sei();

 	while(1)
	{
		handle_serial();

// handle tick
        if(bitRead(TIFR0, TOV0))
        {
            bitSet(TIFR0, TOV0);
            tick++;
            if(tick >= HZ / 10)
            {
                tick = 0;
                LED_TOGGLE

// send to autopilot
                uint8_t text[8];
                text[0] = 0xff;
                text[1] = 0xe5;
                text[2] = encoder_counts & 0xff;
                text[3] = encoder_counts >> 8;
                text[4] = pot_angle & 0xff;
                text[5] = pot_angle >> 8;
                send_uart(text, 6);

// lion readable output
                print_text("DISTANCE: ");
                print_number(encoder_counts);
                print_text("ANGLE: ");
                print_fixed(pot_angle);
                print_text("\n");
            }
        }

        if(bitRead(ADCSRA, ADIF))
        {
            uint8_t channel = ADMUX & 0xf;
            bitSet(ADCSRA, ADIF);
            
// must read the right byte order to update the registers
            uint8_t low = ADCL;
            uint8_t high = ADCH;
            switch(channel)
            {
                case 0:
                    ADMUX = ADMUX_BASE | 1;
                    hall0_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);
                    break;
                case 1:
                    ADMUX = ADMUX_BASE | 2;
                    hall1_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);
                    break;
                case 2:
                    ADMUX = ADMUX_BASE;
                    pot_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);

                    hall_count++;
                    pot_count++;
                    if(pot_count >= POT_OVERSAMPLE)
                    {
                        pot_value = pot_accum / POT_OVERSAMPLE;
                        pot_accum = 0;
                        pot_count = 0;
                        if(pot_value < POT_CENTER)
                        {
                            pot_angle = ((int32_t)POT_CENTER - pot_value) * 
                                45 * FRACTION / 
                                (POT_CENTER - POT_RIGHT);
                        }
                        else
                        if(pot_value > POT_CENTER)
                        {
                            pot_angle = ((int32_t)pot_value - POT_CENTER) *
                                -45 * FRACTION /
                                (POT_LEFT - POT_CENTER); 
                        }
                        else
                            pot_angle = 0;
                    }

                    if(hall_count >= HALL_OVERSAMPLE)
                    {
                        hall0_value = hall0_accum / HALL_OVERSAMPLE;
                        hall1_value = hall1_accum / HALL_OVERSAMPLE;
                        hall_count = 0;
                        hall0_accum = 0;
                        hall1_accum = 0;


                        handle_encoder();

//                         debug_count++;
//                         if(debug_count > 10)
//                         {
//                             debug_count = 0;
//                         print_number(hall0_value);
//                         print_number(hall1_value);
//                         print_number(pot_value);
//                         print_text("\n");
//                        }
                    }
                    break;
            }
        }
    }   
}





