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



// calculate the PWM table on the ATMega or the ARM







// the power
int16_t power = MAX_POWER;
// the motor phase 0 - 255
uint8_t phase = 0;
uint8_t prev_phase = 0;
uint8_t test_counter = 0;

// the PWM program
// 1 for N on, 1 for N off
#define TABLE_SIZE 2
pwm_table_t pwm_table[TABLE_SIZE];
pwm_table_t new_table[TABLE_SIZE];




// create the PWM table
void phase_to_pwm()
{
	uint16_t i;
	if(power < MIN_POWER)
	{
		power = MIN_POWER;
	}


// turn on 1 P & 2 N's
	uint8_t on_delay = 0xff - power;
	uint8_t off_delay = 0xff - (PERIOD - power);
	uint8_t need_deadband = 0;
	if(phase < 85)
	{
		if(prev_phase >= 85)
		{
			need_deadband = 1;
		}
		else
		{
			new_table[0].time = on_delay;
			new_table[0].value = (1 << RX_PIN) | (1 << P_FET1) | (1 << N_FET2) | (1 << N_FET3);
			new_table[1].time = off_delay;
			new_table[1].value = (1 << RX_PIN) | (1 << P_FET1);
		}
	}
	else
	if(phase < 170)
	{
		if(prev_phase >= 170 || prev_phase < 85)
		{
			need_deadband = 1;
		}
		else
		{
			new_table[0].time = on_delay;
			new_table[0].value = (1 << RX_PIN) | (1 << P_FET2) | (1 << N_FET1) | (1 << N_FET3);
			new_table[1].time = off_delay;
			new_table[1].value = (1 << RX_PIN) | (1 << P_FET2);
		}
	}
	else
	{
		if(prev_phase < 170)
		{
			need_deadband = 1;
		}
		else
		{
			new_table[0].time = on_delay;
			new_table[0].value = (1 << RX_PIN) | (1 << P_FET3) | (1 << N_FET1) | (1 << N_FET2);
			new_table[1].time = off_delay;
			new_table[1].value = (1 << RX_PIN) | (1 << P_FET3);
		}
	}

	prev_phase = phase;

#if 1
	if(need_deadband)
	{
//print_text("phase_to_pwm 1 phase=");
//print_number(phase);
//flush_serial();
//send_byte('\n');
// deadband if changing P's
		new_table[0].time = 0xff - DEADBAND;
		new_table[0].value = (1 << RX_PIN);
// dummy
		new_table[1].time = 0xff;
		new_table[1].value = (1 << RX_PIN);


#ifdef __AVR__
		have_new_table = 1;

// wait for deadband to be taken up by the interrupt handler
		while(have_new_table)
		{
		}

#endif // __AVR__


		// generate the proper table
		phase_to_pwm();
	}
	else
#endif

	{


// DEBUG only drive 1 pair
// 	for(i = 0; i < EVENTS; i++)
// 	{
// 		new_table[i].value &= (1 << RX_PIN) | 
// 			(1 << P_FET2) |
// 			(1 << N_FET2);a
// 	}



#ifdef __AVR__
		have_new_table = 1;
#endif // __AVR__
	}

}



void dump_pwm()
{
	print_text("dump_pwm \n");
	print_text("phase=");
	print_number(phase);
	print_text("power=");
	print_number(power);
//	print_text("pwm1=");
//	print_number(pwm1);
//	print_text("pwm2=");
//	print_number(pwm2);
//	print_text("pwm3=");
//	print_number(pwm3);
	flush_serial();
	send_byte('\n');

#if 1
	uint8_t i, j;
	uint16_t period = 0;
	for(i = 0; i < TABLE_SIZE; i++)
	{
		uint16_t diff = 0xff - new_table[i].time;
		period += diff;
		print_text("delay=");
		print_number(diff);
		print_text("\tvalue=");
		
		uint8_t value = new_table[i].value;
		uint8_t buffer[9];
		for(j = 0; j < 8; j++)
		{
			buffer[j] = '-';
		}
		buffer[8] = 0;
		
		if(bitIsSet(value, P_FET1))
		{
			buffer[0] = 'P';
		}
		if(bitIsSet(value, P_FET2))
		{
			buffer[1] = 'P';
		}
		if(bitIsSet(value, P_FET3))
		{
			buffer[2] = 'P';
		}
		if(bitIsSet(value, N_FET1))
		{
			buffer[3] = 'N';
		}
		if(bitIsSet(value, N_FET2))
		{
			buffer[4] = 'N';
		}
		if(bitIsSet(value, N_FET3))
		{
			buffer[5] = 'N';
		}
//		print_bin(new_table[i].value);
		print_text(buffer);
		flush_serial();
		send_byte('\n');
	}

	print_text("period=");
	print_number(period);
	flush_serial();
	send_byte('\n');
#endif
}
