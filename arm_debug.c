#include <stdio.h>
#include "arm_debug.h"







void print_text(const char *string)
{
	printf("%s", string);
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
	if(number > 10000) *ptr++ = '0' + (number / 10000);
	if(number > 1000) *ptr++ = '0' + ((number / 1000) % 10);
	if(number > 100) *ptr++ = '0' + ((number / 100) % 10);
	if(number > 10) *ptr++ = '0' + ((number / 10) % 10);
	*ptr++ = '0' + (number % 10);
	*ptr++ = ' ';
	*ptr = 0;
	print_text(string);
}

void flush_serial()
{
}

void send_byte(unsigned char x)
{
	printf("%c", x);
}
