/*
 * REMOTE CONTROL RECEIVER FOR CAR
 * Copyright (C) 2020-2024 Adam Williams <broadcast at earthling dot net>
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

// receiver using 433Mhz Si4421
// no frequency hopping
// use test_receive.c to view the output

// to program a dead chip with the home made programmer
// enable internal RC + PLL
// /amazon/root/car/usb_programmer -p 18f14k50 -c 0x300000 0011100000000000
// enable watchdog + brownout
// /amazon/root/car/usb_programmer -p 18f14k50 -c 0x300002 0001111100011110
// disable PGM & extended instructions
// /amazon/root/car/usb_programmer -p 18f14k50 -c 0x300006 0000000000000000

// write the hex file
// /amazon/root/car/usb_programmer -p 18f14k50 car_receive.X.production.hex


// PIC18F14K50 Configuration Bit Settings

// 'C' source line config statements


// CONFIG1H
//#pragma config FOSC = IRCCLKOUT // Oscillator Selection bits (Internal RC oscillator, CLKOUT function on OSC2)
#pragma config FOSC = IRC // Oscillator Selection bits (Internal RC oscillator, GPIO on OSC2)
#pragma config PLLEN = ON      // 4 X PLL Enable bit

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 22        // Brown-out Reset Voltage bits (VBOR set to 3.0 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)

// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)



// print debug to the UART.  The ARM has to forward it.
// Required for it to work at all, due to some timing anomaly.
#define DEBUG

// maximum speed for sounds
#define CLOCKSPEED 32000000

// the system clock & the number of packets per second
#define HZ 25
// number of HZ when scanning
#define DWELL_TIME 5

// delay between packets
#define TIMER0_PERIOD (CLOCKSPEED / 4 / 32 / HZ)
// time before next packet we should hop (10ms)
#define HOP_LAG (CLOCKSPEED / 4 / 32 / 100)


const uint8_t PACKET_KEY[] = 
{
    0x5b, 0xb1, 0x6e, 0x6b, 0x33, 0x30, 0x9e, 0x08
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55
};

#define DATA_SIZE 12



#define LED_LAT LATCbits.LATC5
#define LED_TRIS TRISCbits.TRISC5

#define RADIO_CS_LAT LATBbits.LATB4
#define RADIO_CS_TRIS TRISBbits.TRISB4

#define RADIO_SCK_LAT LATCbits.LATC2
#define RADIO_SCK_TRIS TRISCbits.TRISC2

#define RADIO_SDO_LAT LATCbits.LATC1
#define RADIO_SDO_TRIS TRISCbits.TRISC1

//    uint16_t debug;



#define RADIO_DELAY1 0
// delay to warm up the radio is 10ms
#define RADIO_DELAY2 (-CLOCKSPEED / 4 / 4 / 100)
// multiple of clockspeed
#define BAUD 100000


// frequency hopping table.  Use freqs433.ods to calculate.
const uint16_t channels[] = 
{
//    1200, // 433Mhz/desk transmitter
//    1300, // 433.25
    1400, // 433.5Mhz
//    1500, // 433.75Mhz
//    1600, // 434Mhz
    1700, // 434.25Mhz
//    1800  // 434.5Mhz
};

#define TOTAL_CHANNELS (sizeof(channels) / sizeof(uint16_t))

// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code


// scan for synchronous code
#define FIFORSTREG 0xCA81
// read continuously
//#define FIFORSTREG              (0xCA81 | 0x0004)
// 915MHz
//#define FREQ_BAND 0x0030
// 433Mhz
#define FREQ_BAND 0x0010

// Center Frequency page 18
#define CFSREG(chan) (0xA000 | (chan))
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




typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
        unsigned scanning : 1;
        unsigned got_packet : 1;
	};
	
	unsigned char value;
} flags_t;


volatile flags_t flags;
volatile uint32_t tick = 0;
volatile uint8_t current_channel = 0;
volatile uint8_t missed_packets = 0;
#define MAX_MISSED_PACKETS HZ

volatile uint8_t serial_in;
volatile uint8_t key_offset;
volatile uint8_t data_offset;
volatile uint8_t serial_data[DATA_SIZE];
void (*receive_state)();
// mane timer value when the last packet was received
volatile uint16_t start_time;




#define UART_BUFSIZE 64
uint8_t uart_buffer[UART_BUFSIZE];
uint8_t uart_size = 0;
uint8_t uart_position1 = 0;
uint8_t uart_position2 = 0;

uint16_t chars_received = 0;


// send a UART char
void handle_uart()
{
// clear the overflow bit
    if(RCSTAbits.OERR)
    {
        RCSTAbits.OERR = 0;
        RCSTAbits.CREN = 0;
        RCSTAbits.CREN = 1;
    }

    if(uart_size > 0 && PIR1bits.TXIF)
    {
        PIR1bits.TXIF = 0;
        TXREG = uart_buffer[uart_position2++];
		uart_size--;
		if(uart_position2 >= UART_BUFSIZE)
		{
			uart_position2 = 0;
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

void init_radio()
{
// enable outputs
    RADIO_CS_LAT = 1;
    RADIO_CS_TRIS = 0;
    
    RADIO_SDO_LAT = 0;
    RADIO_SDO_TRIS = 0;
    
    RADIO_SCK_LAT = 0;
    RADIO_SCK_TRIS = 0;

// wait a while
    T1CON = 0b10100001;
    TMR1 = RADIO_DELAY1;
    PIR1bits.TMR1IF = 0;
    while(!PIR1bits.TMR1IF)
    {
        ClrWdt();
    }
    T1CON = 0b10000000;
}

void radio_on()
{

// scan for synchronous code
    write_radio(FIFORSTREG);
// enable synchron latch
    write_radio(FIFORSTREG | 0x0002);
    write_radio(GENCREG);
    write_radio(AFCCREG);
    write_radio(CFSREG(channels[current_channel]));
    write_radio(DRVSREG);
    write_radio(PMCREG);
    write_radio(RXCREG);
    write_radio(TXCREG);
    write_radio(BBFCREG);

// turn on the transmitter to warm up.  Not necessary to receive.
//     write_radio(PMCREG | 0x0020);
// // 1:4 prescaler for 32Mhz clock
// 
// 
//     T1CON = 0b10100001;
//     TMR1 = RADIO_DELAY2;
//     PIR1bits.TMR1IF = 0;
//     while(!PIR1bits.TMR1IF)
//     {
//         ClrWdt();
//     }
//     T1CON = 0b10000000;

// receive mode
    write_radio(PMCREG | 0x0080);
}

void print_byte(uint8_t c)
{
	if(uart_size < UART_BUFSIZE)
	{
		uart_buffer[uart_position1++] = c;
		uart_size++;
		if(uart_position1 >= UART_BUFSIZE)
		{
			uart_position1 = 0;
		}
	}
}


#ifdef DEBUG
void print_text(const uint8_t *s)
{
	while(*s != 0)
	{
		print_byte(*s);
		s++;
	}
}


void print_number_nospace(uint16_t number)
{
	if(number >= 10000) print_byte('0' + (number / 10000));
	if(number >= 1000) print_byte('0' + ((number / 1000) % 10));
	if(number >= 100) print_byte('0' + ((number / 100) % 10));
	if(number >= 10) print_byte('0' + ((number / 10) % 10));
	print_byte('0' + (number % 10));
}

void print_number(uint16_t number)
{
    print_number_nospace(number);
   	print_byte(' ');
}

void print_bin(uint8_t number)
{
	print_byte((number & 0x80) ? '1' : '0');
	print_byte((number & 0x40) ? '1' : '0');
	print_byte((number & 0x20) ? '1' : '0');
	print_byte((number & 0x10) ? '1' : '0');
	print_byte((number & 0x8) ? '1' : '0');
	print_byte((number & 0x4) ? '1' : '0');
	print_byte((number & 0x2) ? '1' : '0');
	print_byte((number & 0x1) ? '1' : '0');
}
#else // DEBUG
void print_text(const uint8_t *s) {}
void print_number_nospace(uint16_t number) {}
void print_number(uint16_t number) {}
void print_bin(uint8_t number) {}

#endif // !DEBUG



void serial_on()
{
// serial port
    TXSTA = 0b00100100;
    RCSTA = 0b10010000;
    BAUDCON = 0b00001000;
// baud = clockspeed / (4 * (SPBRG + 1))
    SPBRG = CLOCKSPEED / 4 / BAUD - 1;
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 1;
}

void get_key();
void get_data()
{
// get data from radio
    serial_data[data_offset++] = serial_in;
    if(data_offset >= DATA_SIZE)
    {
        key_offset = 0;
        receive_state = get_key;
        
        uint8_t i;
        uint8_t failed = 0;

// XOR the data key
        for(i = 0; i < DATA_SIZE; i++)
        {
            serial_data[i] ^= DATA_KEY[i];
        }


        for(i = 3; i < DATA_SIZE; i += 3)
        {
            if(serial_data[0] != serial_data[i] ||
                serial_data[1] != serial_data[i + 1] ||
                serial_data[2] != serial_data[i + 2])
            {
// reject packet if any value is different
                failed = 1;
                break;
            }
        }

// valid packet
        if(!failed)
        {
            LED_LAT = !LED_LAT;
// cue the bottom half to send values to the host
            flags.got_packet = 1;
            flags.scanning = 0;
            missed_packets = 0;

            uint8_t code_byte = serial_data[2];
            current_channel = ((code_byte >> 5) & 0x3);
        }
    }
}

void get_key()
{
// get packet key from radio
    if(serial_in == PACKET_KEY[key_offset])
    {
        if(key_offset == 0)
        {
            start_time = TMR0;
        }

        key_offset++;
        if(key_offset >= sizeof(PACKET_KEY))
        {
            receive_state = get_data;
            data_offset = 0;
            key_offset = 0;
        }
    }
    else
    if(serial_in == PACKET_KEY[0])
    {
        key_offset = 1;
        start_time = TMR0;
    }
    else
    if(key_offset > 0)
    {
        key_offset = 0;
    }
}


void main()
{
    OSCCON = 0b11100000;


// LED on
    LED_LAT = 0;
    LED_TRIS = 0;
    ANSEL = 0b00000000;
    ANSELH = 0b00000000;

	flags.value = 0;
    tick = 0;
    key_offset = 0;
    receive_state = get_key;

    init_radio();
    radio_on();
    serial_on();

// mane timer
// 1:32 prescaler for 32Mhz clock
    T0CON = 0b10000100;
    TMR0 = -TIMER0_PERIOD;

    INTCON = 0b11100000;

    while(1)
    {
        handle_uart();

// guaranteed to fire before the radio gets the next packet
        if(flags.got_packet)
        {
            flags.got_packet = 0;
            print_byte(0xff);
            print_byte(0xe7);
            print_byte(serial_data[0]);
            print_byte(serial_data[1]);
            print_byte(serial_data[2]);
        }
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
			flags.interrupt_complete = 0;
			INTCONbits.TMR0IF = 0;
            TMR0 = -TIMER0_PERIOD;
            tick++;
        }

// serial port
        if(PIR1bits.RCIF)
        {
            flags.interrupt_complete = 0;
//            LED_LAT = !LED_LAT;
            serial_in = RCREG;
            PIR1bits.RCIF = 0;
//            chars_received++;
            receive_state();
        }
        
		if(flags.interrupt_complete) break;
    }
}










