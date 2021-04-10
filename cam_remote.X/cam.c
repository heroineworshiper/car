/*
 * REMOTE CONTROL FOR CAM PANNER
 * Copyright (C) 2020 Adam Williams <broadcast at earthling dot net>
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



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <xc.h>
#include <pic18f14k50.h>




// PIC18F14K50 Configuration Bit Settings

// 'C' source line config statements


// CONFIG1H
//#pragma config FOSC = IRCCLKOUT // Oscillator Selection bits (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config FOSC = IRC // Oscillator Selection bits (Internal RC oscillator, GPIO on OSC2)
#pragma config PLLEN = OFF      // 4 X PLL Enable bit (PLL is under software control)

// CONFIG2L
#pragma config PWRTEN = ON     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 22        // Brown-out Reset Voltage bits (VBOR set to 3.0 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)

// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)




// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// based on the maximum timer values
#define CLOCKSPEED 2000000

// the system clock & the number of packets per second
#define PACKET_HZ 25

// delay between packets
#define TIMER0_VALUE (-CLOCKSPEED / 4 / PACKET_HZ)



const uint8_t PACKET_KEY[] = 
{
    0xff, 0xe7, 0x2f, 0x38, 0x73, 0xae, 0x5e, 0x90
};


#define LED_LAT LATCbits.LATC7
#define LED_TRIS TRISCbits.TRISC7

#define RADIO_CS_LAT LATCbits.LATC6
#define RADIO_CS_TRIS TRISCbits.TRISC6

#define RADIO_SDO_LAT LATCbits.LATC4
#define RADIO_SDO_TRIS TRISCbits.TRISC4

#define RADIO_SCK_LAT LATCbits.LATC3
#define RADIO_SCK_TRIS TRISCbits.TRISC3

//    uint16_t debug;



// delay to warm up the radio is 10ms
#define RADIO_DELAY (-CLOCKSPEED / 4 / 100)
// number of times to resend the packet
#define RESENDS 4
// multiple of clockspeed
#define BAUD 100000


// RADIO_CHANNEL is from 96-3903 & set by the user
// Frequency in Mhz is 900 + 30 * RADIO_CHANNEL / 4000
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

#define RADIO_CHANNEL 96

// scan for synchronous code
#define FIFORSTREG 0xCA81
// read continuously
//#define FIFORSTREG              (0xCA81 | 0x0004)
// 915MHz
#define FREQ_BAND 0x0030
// Center Frequency: 915.000MHz
#define CFSREG (0xA000 | RADIO_CHANNEL)
// crystal load 10pF
#define XTAL_LD_CAP 0x0003
// power management page 16
#define PMCREG 0x8201
#define GENCREG (0x8000 | XTAL_LD_CAP | FREQ_BAND)


// +3/-4 Fres
//#define AFCCREG 0xc4f7
// +15/-16 Fres
#define AFCCREG 0xc4d7

// Data Rate
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) - 1
#define RADIO_BAUD_CODE 3

// data rate prescaler.  Divides data rate by 8 if 1
//#define DRPE (1 << 7)
#define DRPE 0
#define DRVSREG (0xC600 | DRPE | RADIO_BAUD_CODE)


// Page 37 of the SI4421 datasheet gives optimum bandwidth values
// but the lowest that works is 200khz
//#define RXCREG 0x9481     // BW 200KHz, LNA gain 0dB, RSSI -97dBm
//#define RXCREG 0x9440     // BW 340KHz, LNA gain 0dB, RSSI -103dBm
#define RXCREG 0x9420       // BW 400KHz, LNA gain 0dB, RSSI -103dBm

//#define TXCREG 0x9850     // FSK shift: 90kHz
#define TXCREG 0x98f0       // FSK shift: 165kHz
#define STSREG 0x0000
#define RXFIFOREG 0xb000

// analog filter for raw mode
#define BBFCREG                 0xc23c


// C * 10 to resistance in ohms
typedef struct
{
    int16_t t;
    uint16_t adc;
} table_t;




typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
        unsigned have_stick : 1;
        unsigned done : 1;
	};
	
	unsigned char value;
} flags_t;

#define TIMELAPSE_OFF 0x3
#define TIMELAPSE_FAST_LEFT 0x0
#define TIMELAPSE_FAST_RIGHT 0x1

#define LED_TICKS (PACKET_HZ / 5)

// analog thresholds for timelapse mode
#define FAST_RIGHT 0x40
#define FAST_LEFT 0xc0
// deadband values from the ARM for starting the transmitter
#define MIN_LEFT 130
#define MIN_RIGHT 126 

// Each byte is a number of LED_TICKS between LED toggles
#define BLINK_PATTERN_SIZE 6
const uint8_t blink_patterns[] = 
{
//  on  off on  off on  off 
//    3,  1,  1,  1,  1,  4,  // fast left timelapse
    3,  1,  1,  4,  0,  0,  // slow left timelapse
    1,  1,  3,  4,  0,  0,  // slow right timelapse
    1, 1, 1, 1, 1, 1, // no timelapse
    1, 1, 1, 1, 1, 1, // no timelapse
//    1,  1,  1,  1,  3,  4   // fast right timelapse
};

flags_t flags;
uint32_t tick;
uint32_t prev_tick;
uint8_t timelapse_mode = TIMELAPSE_OFF;
uint8_t blink_offset = 0;
uint8_t blink_counter = 0;
// hall effect accumulator
uint32_t adc_accum;
uint32_t adc_count;

void powerup();
void get_timelapse();
void wait_timelapse();
void (*adc_state)() = powerup;

// called every tick
void handle_led()
{
    if(adc_state == powerup ||
        adc_state == get_timelapse)
    {
// do nothing until timelapse code is known
    }
    else
    if(timelapse_mode == TIMELAPSE_OFF)
    {
        LED_LAT = !LED_LAT;
    }
    else
    if((tick % LED_TICKS) == 0)
    {
        const uint8_t *blink_pattern = blink_patterns + 
            timelapse_mode * BLINK_PATTERN_SIZE;

        if(blink_counter > 0)
        {
            blink_counter--;
        }
        if(blink_counter == 0)
        {
            LED_LAT = !LED_LAT;
            blink_offset++;
            if(blink_offset >= BLINK_PATTERN_SIZE)
            {
                blink_offset = 0;
            }

// skip 0 slots
            while(blink_pattern[blink_offset] == 0)
            {
                blink_offset++;
                if(blink_offset >= BLINK_PATTERN_SIZE)
                {
                    blink_offset = 0;
                }
            }

            blink_counter = blink_pattern[blink_offset];
        }
    }

}


void write_radio(uint16_t data)
{
    ClrWdt();

    RADIO_CS_LAT = 0;
    uint8_t i;
    for(i = 0; i < 16; i++)
    {
        RADIO_SDO_LAT = (uint8_t)((data & 0x8000) ? 1 : 0);
        data <<= 1;
        RADIO_SCK_LAT = 1;
        RADIO_SCK_LAT = 0;
    }
    RADIO_CS_LAT = 1;
}

void radio_on()
{
// enable outputs
    RADIO_CS_LAT = 1;
    RADIO_CS_TRIS = 0;
    
    RADIO_SDO_LAT = 0;
    RADIO_SDO_TRIS = 0;
    
    RADIO_SCK_LAT = 0;
    RADIO_SCK_TRIS = 0;
    
// scan for synchronous code
    write_radio(FIFORSTREG);
// enable synchron latch
    write_radio(FIFORSTREG | 0x0002);
    write_radio(GENCREG);
    write_radio(AFCCREG);
    write_radio(CFSREG);
    write_radio(DRVSREG);
    write_radio(PMCREG);
    write_radio(RXCREG);
    write_radio(TXCREG);
    write_radio(BBFCREG);
// turn on the transmitter to tune
    write_radio(PMCREG | 0x0020);

// warm up
//    LED_LAT = 0;
    T1CON = 0b10000001;
    TMR1 = RADIO_DELAY;
    PIR1bits.TMR1IF = 0;


// do other stuff during the delay
//    handle_led();

    while(!PIR1bits.TMR1IF)
    {
        ClrWdt();
    }
    T1CON = 0b10000000;
//    LED_LAT = 1;
}

void radio_off()
{
    RADIO_CS_LAT = 1;
    RADIO_CS_TRIS = 0;
    
    RADIO_SDO_LAT = 0;
    RADIO_SDO_TRIS = 0;
    
    RADIO_SCK_LAT = 0;
    RADIO_SCK_TRIS = 0;
    
// disable radio
    write_radio(PMCREG);

    RADIO_CS_TRIS = 1;
    RADIO_SDO_TRIS = 1;
    RADIO_SCK_TRIS = 1;
}

void serial_on()
{
// serial port
    TXSTA = 0b00100100;
    RCSTA = 0b10000000;
    BAUDCON = 0b00001000;
// baud = clockspeed / (4 * (SPBRG + 1))
    SPBRG = CLOCKSPEED / 4 / BAUD - 1;
}

void flush_serial()
{
    ClrWdt();
    while(!PIR1bits.TXIF)
    {
    }

    while(!TXSTAbits.TRMT)
    {
    }
}

void serial_off()
{
    flush_serial();

    TXSTA = 0x0;
    RCSTA = 0x0;
}

void write_serial(uint8_t value)
{
    ClrWdt();

    while(!PIR1bits.TXIF)
    {
    }

    TXREG = value;    
}


void get_stick()
{
    flags.have_stick = 1;
}

// wait for stick to be released to prevent timelapse selection from moving motor
void wait_timelapse()
{
// LED now requires a free running tick
    if(tick > prev_tick)
    {
        uint8_t value = adc_accum / adc_count / 4;
        if(value <= MIN_LEFT &&
            value >= MIN_RIGHT)
        {
            adc_state = get_stick;
        }
        
        adc_accum = 0;
        adc_count = 0;
        prev_tick = tick;
    }
}

// get timelapse code
void get_timelapse()
{
    if(tick >= PACKET_HZ / 5)
    {
        uint8_t value = adc_accum / adc_count / 4;
        if(value <= FAST_RIGHT)
        {
            timelapse_mode = TIMELAPSE_FAST_RIGHT;
        }
        else
        if(value >= FAST_LEFT)
        {
            timelapse_mode = TIMELAPSE_FAST_LEFT;
        }

        adc_accum = 0;
        adc_count = 0;
        adc_state = wait_timelapse;


// start the LED for the timelapse code
        const uint8_t *blink_pattern = blink_patterns + 
            timelapse_mode * BLINK_PATTERN_SIZE;
        blink_counter = blink_pattern[0];
        LED_LAT = 0;
        tick = 0;
        prev_tick = 0;
    }
}

// wait for voltages to rise
void powerup()
{
    if(tick >= PACKET_HZ / 5)
    {
        adc_accum = 0;
        adc_count = 0;
        adc_state = get_timelapse;
        tick = 0;
    }
}


void main()
{
    OSCCON = 0b11000000;


// LED off until initial button state is captured
    LED_LAT = 1;
    LED_TRIS = 0;

	flags.value = 0;
    tick = 0;


// start ADC
// AN4 is the hall effect sensor
    ANSEL = 0b00010000;
    ANSELH = 0b00000000;
    ADCON0 = 0b00010001;
    ADCON2 = 0b10111110;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
// off during initialization
    LED_LAT = 1;
    
    radio_off();


// mane timer
    T0CON = 0b10001000;
    TMR0 = TIMER0_VALUE;

    INTCON = 0b11100000;

// start getting stick values
    ADCON0bits.GO = 1;


    while(1)
    {
        ClrWdt();
    }
    
}

void interrupt isr()
{
	while(1)
	{
		ClrWdt();
		flags.interrupt_complete = 1;

// tick counter
		if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
            TMR0 = TIMER0_VALUE;
			flags.interrupt_complete = 0;
            tick++;

            handle_led();
            if(flags.have_stick)
            {
// transmit packet
                radio_on();
                serial_on();

                uint8_t adc_value;
                adc_value = adc_accum / adc_count / 4;
                adc_accum = 0;
                adc_count = 0;

                uint8_t i;
                for(i = 0; i < sizeof(PACKET_KEY); i++)
                {
                    write_serial(PACKET_KEY[i]);
                }
                write_serial(timelapse_mode);
                write_serial(adc_value);
                write_serial(timelapse_mode);
                write_serial(adc_value);
                write_serial(timelapse_mode);
                write_serial(adc_value);
                write_serial(timelapse_mode);
                write_serial(adc_value);

// DEBUG
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);

                serial_off();
                radio_off();

            }
        }


// ADC
        if(PIR1bits.ADIF)
        {
            flags.interrupt_complete = 0;
            adc_accum += ADRES;
            adc_count++;
            PIR1bits.ADIF = 0;
            ADCON0bits.GO = 1;
            
            
            adc_state();
        }


		if(flags.interrupt_complete) break;
    }
}










