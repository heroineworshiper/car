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



// calculate the PWM table on the ATMega or the ARM




// created by tables.c
const uint8_t sin_table[] = {
	0x80, 0x83, 0x86, 0x89, 0x8c, 0x8f, 0x92, 0x95, 
	0x98, 0x9c, 0x9f, 0xa2, 0xa5, 0xa8, 0xab, 0xae, 
	0xb0, 0xb3, 0xb6, 0xb9, 0xbc, 0xbf, 0xc1, 0xc4, 
	0xc7, 0xc9, 0xcc, 0xce, 0xd1, 0xd3, 0xd5, 0xd8, 
	0xda, 0xdc, 0xde, 0xe0, 0xe2, 0xe4, 0xe6, 0xe8, 
	0xea, 0xec, 0xed, 0xef, 0xf0, 0xf2, 0xf3, 0xf5, 
	0xf6, 0xf7, 0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfc, 
	0xfd, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 
	0xfd, 0xfc, 0xfc, 0xfb, 0xfa, 0xf9, 0xf8, 0xf7, 
	0xf6, 0xf5, 0xf3, 0xf2, 0xf0, 0xef, 0xed, 0xec, 
	0xea, 0xe8, 0xe6, 0xe4, 0xe2, 0xe0, 0xde, 0xdc, 
	0xda, 0xd8, 0xd5, 0xd3, 0xd1, 0xce, 0xcc, 0xc9, 
	0xc7, 0xc4, 0xc1, 0xbf, 0xbc, 0xb9, 0xb6, 0xb3, 
	0xb0, 0xae, 0xab, 0xa8, 0xa5, 0xa2, 0x9f, 0x9c, 
	0x98, 0x95, 0x92, 0x8f, 0x8c, 0x89, 0x86, 0x83, 
	0x80, 0x7c, 0x79, 0x76, 0x73, 0x70, 0x6d, 0x6a, 
	0x67, 0x63, 0x60, 0x5d, 0x5a, 0x57, 0x54, 0x51, 
	0x4f, 0x4c, 0x49, 0x46, 0x43, 0x40, 0x3e, 0x3b, 
	0x38, 0x36, 0x33, 0x31, 0x2e, 0x2c, 0x2a, 0x27, 
	0x25, 0x23, 0x21, 0x1f, 0x1d, 0x1b, 0x19, 0x17, 
	0x15, 0x13, 0x12, 0x10, 0x0f, 0x0d, 0x0c, 0x0a, 
	0x09, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x03, 
	0x02, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 
	0x02, 0x03, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 
	0x09, 0x0a, 0x0c, 0x0d, 0x0f, 0x10, 0x12, 0x13, 
	0x15, 0x17, 0x19, 0x1b, 0x1d, 0x1f, 0x21, 0x23, 
	0x25, 0x27, 0x2a, 0x2c, 0x2e, 0x31, 0x33, 0x36, 
	0x38, 0x3b, 0x3e, 0x40, 0x43, 0x46, 0x49, 0x4c, 
	0x4f, 0x51, 0x54, 0x57, 0x5a, 0x5d, 0x60, 0x63, 
	0x67, 0x6a, 0x6d, 0x70, 0x73, 0x76, 0x79, 0x7c
};



#define SIN_TABLE_SIZE (sizeof(sin_table))


#define EVENTS 7
event_table_t event_table[EVENTS];
uint8_t total_events;

// the power
int16_t power = MIN_POWER;
// the motor phase 0 - 255
uint8_t phase = 0;

// pwm values derived from the motor phase 0 - 255
int32_t pwm1;
int32_t pwm2;
int32_t pwm3;

// the PWM program
// 1 for overflow, 6 for FET changes
#define TABLE_SIZE 8
pwm_table_t pwm_table[TABLE_SIZE];
pwm_table_t new_table[TABLE_SIZE];


volatile uint8_t have_new_table = 0;
// entry in pwm_table we're playing back
pwm_table_t *pwm_ptr;


// timer 2 overflow
ISR(TIMER2_OVF_vect)
{
// set the FET gates
	PORTD = 
// don't overwrite the serial port TX pin
#ifndef DO_RECEIVE
        (PORTD & (1 << TX_PIN)) | 
#endif
        pwm_ptr->value;
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
// enable the overflow interrupt & disable the FETs
	bitSet(TIMSK, TOIE2);
	PORTD = 
#ifdef DO_RECEIVE
        (1 << RX_PIN) |
#endif
        0;
}

void pwm_off()
{
// disable the overflow interrupt & enable the N FETS for braking
	bitClear(TIMSK, TOIE2);
	PORTD = 
#ifdef DO_RECEIVE
        (1 << RX_PIN) |
#endif
        (1 << N_FET1) |
        (1 << N_FET2) |
        (1 << N_FET3);
}


void put_event(uint16_t time, uint8_t p_mask, uint8_t n_mask)
{
	uint8_t i, j;
	for(i = 0; i < total_events; i++)
	{
		if(event_table[i].time > time)
		{
// push back later events
			for(j = total_events - 1; 1; j--)
			{
				event_table[j + 1] = event_table[j];
// can never be <= i
				if(j == i) break;
			}
			break;
		}
	}

// store new event
	event_table[i].time = time;
	event_table[i].p_mask = p_mask;
	event_table[i].n_mask = n_mask;
	total_events++;
}

// create the PWM table
void phase_to_pwm()
{
	uint8_t i;

	if(power < MIN_POWER)
	{
		power = MIN_POWER;
	}


// absolute times of FET switching
	pwm1 = ((int32_t)sin_table[(uint8_t)(phase)]) * 
		power / 
		0xff;
	pwm2 = ((int32_t)sin_table[(uint8_t)(phase + 85)]) * 
		power / 
		0xff;
	pwm3 = ((int32_t)sin_table[(uint8_t)(phase + 170)]) * 
		power / 
		0xff;

// DEBUG
//pwm1 = 32;
//pwm2 = 64;
//pwm3 = 1;

// make the temporary table
	total_events = 0;
// turn on all the P's
	put_event(0, 0xff, 0);
//	put_event(0, 0, 0);

// switch 1 to N
	put_event(pwm1, (uint8_t)(~(1 << P_FET1)), 0);
//	put_event(pwm1, 0, 0);
	put_event(pwm1 + DEADBAND, 0xff, (1 << N_FET1));
//	put_event(pwm1 + DEADBAND, 0xff, 0);

// switch 2 to N
	put_event(pwm2, (uint8_t)(~(1 << P_FET2)), 0);
//	put_event(pwm2, 0, 0);
	put_event(pwm2 + DEADBAND, 0xff, (1 << N_FET2));
//	put_event(pwm2 + DEADBAND, 0xff, 0);

// switch 3 to N
	put_event(pwm3, (uint8_t)(~(1 << P_FET3)), 0);
//	put_event(pwm3, 0, 0);
	put_event(pwm3 + DEADBAND, 0xff, (1 << N_FET3));
//	put_event(pwm3 + DEADBAND, 0xff, 0);

//	dump_events();


// RX_PIN must always be high to enable the pullup
	uint8_t current_port = 
#ifdef DO_RECEIVE
        (1 << RX_PIN) |
#endif
		(1 << P_FET1) |
		(1 << P_FET2) |
		(1 << P_FET3);
	pwm_table_t *ptr = &new_table[0];
	uint8_t new_time;
	uint8_t new_value;
	for(i = 0; i < EVENTS; i++)
	{
		event_table_t *event_ptr = &event_table[i];
		current_port &= event_ptr->p_mask;
		current_port |= event_ptr->n_mask;
		
		uint8_t diff = 0;
		if(i < EVENTS - 1)
		{
			diff = (event_table[i + 1].time - event_ptr->time);
			ptr->time = 0xff - diff;
		}
		else
		if(event_ptr->time > (int16_t)PERIOD)
		{
			diff = 0;
		}
		else
		{
			diff = (int16_t)PERIOD - event_ptr->time;
		}
		
		ptr->time = 0xff - diff;
		ptr->value = current_port;
		ptr++;
	}
	
// deadband with all FETs off saves bugger all current
	ptr->time = 0xff - DEADBAND2;
#ifdef DO_RECEIVE
	ptr->value = (1 << RX_PIN);
#else
	ptr->value = 0;
#endif






// DEBUG only drive 1 pair
// 	for(i = 0; i < EVENTS; i++)
// 	{
// 		new_table[i].value &= (1 << RX_PIN) | 
// 			(1 << P_FET2) |
// 			(1 << N_FET2);
// 	}



#ifdef __AVR__
	have_new_table = 1;
#endif // __AVR__


}


void dump_events()
{
//	print_text("dump_events:\n");
	uint8_t i;

#if 0
// dump a full listing of the events
 	for(i = 0; i < total_events; i++)
 	{
 		print_text("time=");
 		print_number(event_table[i].time);
 		print_text("\tp_mask=");
 		print_bin(event_table[i].p_mask);
 		print_text("\tn_mask=");
 		print_bin(event_table[i].n_mask);
 		flush_serial();
 		send_byte('\n');
 	}
#endif

#if 1
// dump the times of the events	
	for(i = 0; i < total_events; i++)
	{
		print_number(event_table[i].time);
	}

	flush_serial();
	send_byte('\n');
#endif

}



void dump_pwm()
{
	print_text("dump_pwm \n");
	flush_serial();
	print_text("phase=");
	print_number(phase);
	print_text("power=");
	print_number(power);
	print_text("pwm1=");
	print_number(pwm1);
	print_text("pwm2=");
	print_number(pwm2);
	print_text("pwm3=");
	print_number(pwm3);
	flush_serial();
	send_byte('\n');

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
}

void dump_delays()
{
	print_number(phase);
	uint8_t i;
	for(i = 0; i < EVENTS - 1; i++)
	{
		int16_t diff = event_table[i + 1].time - event_table[i].time;
		print_number(diff);
	}
	int16_t diff = (int16_t)PERIOD - event_table[EVENTS - 1].time;
	print_number(diff);
	send_byte('\n');
}


void init_pwm()
{
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
}






