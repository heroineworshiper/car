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





// ARM segment of leg controller
// compile with make leg


#include "leg.h"

void delayMicroseconds(unsigned int us)
{
}

void workaround()
{
}

#include "pwm_routines.c"

void main()
{
	int i;


	power = MAX_POWER;
	phase = 0;

#if 0
	phase_to_pwm();
	dump_events();
	dump_pwm();
#endif


#if 1
	for(i = 0; i < 512; i++)
	{
		phase = i;
//		phase = 235;
		phase_to_pwm();
//		dump_events();
//		dump_delays();
		dump_pwm();
	}
#endif

}








