/*
 * Smart leash
 * Copyright (C) 2022-2025 Adam Williams <broadcast at earthling dot net>
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

// Plug in the ISP programmer
// make leash.hex
// make leash_fuse
// make leash_isp

// Arduino ISP to leash:
// GND - GND
// 13 - CLK
// 12 - MISO
// 11 - MOSI
// 10 - RESET


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
#define LED_DOWNSAMPLE 15

#define BAUD 115200L

#define ENCODER_CENTER 128
#define ENCODER_THRESHOLD 8
#define ON_THRESHOLD 100

#define ENCODER_START 0
#define ENCODER0_LOW 1
#define ENCODER1_LOW 2
#define ENCODER0_HIGH 3
#define ENCODER1_HIGH 4
int8_t encoder_state = ENCODER_START;
int8_t prev_encoder = ENCODER_START;
int16_t encoder_counts = 0;
uint8_t debug_count = 0;

uint16_t enable_accum = 0;
uint16_t encoder0_accum = 0;
uint16_t encoder1_accum = 0;
uint16_t angle_accum = 0;
uint8_t enable_adc = 0;
uint8_t encoder0_adc = 0;
uint8_t encoder1_adc = 0;
uint8_t angle_adc = 0;
uint8_t encoder_count = 0;
uint8_t angle_count = 0;
uint8_t enable_count = 0;
#define ENCODER_OVERSAMPLE 1
#define POT_OVERSAMPLE 32
#define ENABLE_OVERSAMPLE 32
uint8_t tick;
uint8_t led_count = 0;

// byte stuffing for UART
#define START_CODE 0xff
#define ESC_CODE 0xfe

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

void send_uart1(uint8_t c)
{
    if(uart_used < UART_SIZE)
    {
        uart_buffer[uart_write_ptr++] = c;
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


void encode_serial(uint8_t c)
{
    if(c == START_CODE || c == ESC_CODE)
    {
        send_uart1(ESC_CODE);
        send_uart1(0xff ^ c);
    }
    else
        send_uart1(c);
}

void encode_start()
{
    send_uart1(START_CODE);
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
    if(encoder1_adc <= ENCODER_CENTER - ENCODER_THRESHOLD)
    {
        encoder_state = ENCODER1_LOW;
    }
    else
    if(encoder1_adc >= ENCODER_CENTER + ENCODER_THRESHOLD)
    {
        encoder_state = ENCODER1_HIGH;
    }
    else
    if(encoder0_adc <= ENCODER_CENTER - ENCODER_THRESHOLD)
    {
        encoder_state = ENCODER0_LOW;
    }
    else
    if(encoder0_adc >= ENCODER_CENTER + ENCODER_THRESHOLD)
    {
        encoder_state = ENCODER0_HIGH;
    }
    
    if(encoder_state != prev_encoder)
    {
        if(prev_encoder == ENCODER1_LOW && encoder_state == ENCODER0_LOW ||
            prev_encoder == ENCODER0_LOW && encoder_state == ENCODER1_HIGH ||
            prev_encoder == ENCODER1_HIGH && encoder_state == ENCODER0_HIGH ||
            prev_encoder == ENCODER0_HIGH && encoder_state == ENCODER1_LOW)
            encoder_counts++;
        else
        if(prev_encoder == ENCODER0_LOW && encoder_state == ENCODER1_LOW ||
            prev_encoder == ENCODER1_LOW && encoder_state == ENCODER0_HIGH ||
            prev_encoder == ENCODER0_HIGH && encoder_state == ENCODER1_HIGH ||
            prev_encoder == ENCODER1_HIGH && encoder_state == ENCODER0_LOW)
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
// disable digital inputs
    DIDR0 = 0b00001111;
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
            if(tick >= HZ / 25)
            {
                tick = 0;
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
                    encoder0_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);
                    break;
                case 1:
                    ADMUX = ADMUX_BASE | 2;
                    encoder1_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);
                    encoder_count++;

                    if(encoder_count >= ENCODER_OVERSAMPLE)
                    {
                        encoder0_adc = encoder0_accum / ENCODER_OVERSAMPLE / 4;
                        encoder1_adc = encoder1_accum / ENCODER_OVERSAMPLE / 4;
                        encoder_count = 0;
                        encoder0_accum = 0;
                        encoder1_accum = 0;


                        handle_encoder();

//                         debug_count++;
//                         if(debug_count > 10)
//                         {
//                             debug_count = 0;
//                         print_number(encoder0_adc);
//                         print_number(encoder1_adc);
//                         print_number(angle_adc);
//                         print_text("\n");
//                        }
                    }
                    break;
                case 2:
                    ADMUX = ADMUX_BASE | 3;
                    angle_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);

                    angle_count++;
                    if(angle_count >= POT_OVERSAMPLE)
                    {
// scale it to maximize resolution
                        angle_accum -= 32 * 512;
                        angle_accum /= POT_OVERSAMPLE;
                        angle_accum += 128;
                        angle_adc = angle_accum;
//                        angle_adc = angle_accum / POT_OVERSAMPLE / 4;
                        angle_accum = 0;
                        angle_count = 0;

// send to autopilot
                        if(enable_adc >= ON_THRESHOLD)
                        {
                            LED_TOGGLE

                            encode_start();
                            encode_serial(encoder_counts & 0xff);
                            encode_serial(encoder_counts >> 8);
                            encode_serial(angle_adc);
                            encode_serial(encoder0_adc);
                            encode_serial(encoder1_adc);
                        }
                        else
                        {
// standby
                            encoder_counts = 0;
                            led_count++;
                            if(led_count >= LED_DOWNSAMPLE)
                            {
                                led_count = 0;
                                LED_TOGGLE
                            }
                        }
                    }
                    break;

                case 3:
                    ADMUX = ADMUX_BASE;
                    enable_accum += (high << 8) | low;
                    bitSet(ADCSRA, ADSC);
                    enable_count++;

                    if(enable_count >= ENABLE_OVERSAMPLE)
                    {
                        enable_adc = enable_accum / ENABLE_OVERSAMPLE / 4;
                        enable_accum = 0;
                        enable_count = 0;

// lion readable output
                        print_text("DEBUG: ");
                        print_number(enable_adc);
                        print_number(angle_adc);
                        print_number(encoder0_adc);
                        print_number(encoder1_adc);
                        print_number(encoder_counts);
                        print_text("\n");
                    }
                    break;
            }
        }
    }
}





