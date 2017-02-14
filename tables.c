
// construct tables

#include "stdio.h"
#include "stdlib.h"
#include <math.h>
#include "stdint.h"

#define N_SIN 256
int table[N_SIN];



int sin_table()
{
	int i;
	printf("// created by tables.c\n");
	printf("const uint8_t sin_table[] = {\n\t");
	for(i = 0; i < 256; i++)
	{
		int value = (int)(sin((float)i / 256 * 2 * M_PI) * 0x80 + 0x80);
		if(value > 0xff) value = 0xff;
// limit it to prevent PWM glitches
		if(value < 0x1) value = 0x1;
		printf("0x%02x", value);
		if(i < 255) printf(", ");
		if(!((i + 1) % 8) && i < 255) printf("\n\t");
	}
	printf("\n};\n");
}



int main()
{
	sin_table();
}






