
// construct tables
// gcc -o tables tables.c -lm;./tables > sin_table.h

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#include "motor.h"

int table[SIN_TOTAL];


int sin_table()
{
	int i;
	printf("// created by tables.c\n");
	printf("int8_t code sin_table[] = {\n\t");
	for(i = 0; i < SIN_TOTAL; i++)
	{
        float value_f = sin((float)i / SIN_TOTAL * 2 * M_PI);
		int value = (int)(value_f * 127);

        if(value < -127)
            value = -127;
        if(value > 128)
            value = 128;

//        printf("%d\n", value & 0xffff);

		printf("%d", value);
		if(i < SIN_TOTAL - 1) printf(", ");
		if(!((i + 1) % 8) && i < SIN_TOTAL - 1) printf("\n\t");

	}
	printf("\n};\n");
}



int main()
{
	sin_table();
}






