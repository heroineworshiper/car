/*
 * REMOTE CONTROL TRANSMITTER FOR CAR
 * Copyright (C) 2020-2021 Adam Williams <broadcast at earthling dot net>
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
#pragma config PLLEN = ON      // 4 X PLL Enable bit

// CONFIG2L
#pragma config PWRTEN = ON     // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 22        // Brown-out Reset Voltage bits (VBOR set to 2.2 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bit (WDT is always enabled. SWDTEN bit has no effect.)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)

// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)




// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// maximum speed for sounds
#define CLOCKSPEED 32000000

// the system clock & the number of packets per second
#define HZ 25

// delay between packets
#define TIMER0_VALUE (-CLOCKSPEED / 4 / 32 / HZ)



const uint8_t PACKET_KEY[] = 
{
    0x5b, 0xb1, 0x6e, 0x6b, 0x33, 0x30, 0x9e, 0x08
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55
};



// AN4 steering
// AN5 throttle
// C5 speed+
// C5 speed-
// A5, A4 speaker

#define SPEAKER_LAT1 LATAbits.LATA5
#define SPEAKER_LAT2 LATAbits.LATA4
#define SPEAKER_TRIS1 TRISAbits.TRISA5
#define SPEAKER_TRIS2 TRISAbits.TRISA4
#define INCREASE_PORT PORTCbits.RC5
#define DECREASE_PORT PORTCbits.RC6

#define LED_LAT LATCbits.LATC7
#define LED_TRIS TRISCbits.TRISC7

#define AMP_LAT LATCbits.LATC4
#define AMP_TRIS TRISCbits.TRISC4


// B5 is stuck high when it's an output
#define RADIO_CS_LAT LATBbits.LATB6
#define RADIO_CS_TRIS TRISBbits.TRISB6

#define RADIO_SCK_LAT LATBbits.LATB4
#define RADIO_SCK_TRIS TRISBbits.TRISB4

#define RADIO_SDO_LAT LATCbits.LATC2
#define RADIO_SDO_TRIS TRISCbits.TRISC2

//    uint16_t debug;



// delay to warm up the radio is 10ms
#define RADIO_DELAY (-CLOCKSPEED / 4 / 4 / 100)
// multiple of clockspeed
#define BAUD 100000


// RADIO_CHANNEL is from 96-3903 & set by the user
// Frequency in Mhz is 900 + 30 * RADIO_CHANNEL / 4000

// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

// frequency hopping table.  64 frequency steps = 480khz 
// Check temp_sensor.X & cam_remote.X for taken frequencies
// 901-928Mhz
#define MAX_FREQ 3839
#define MIN_FREQ 160
#define FREQ_RANGE (MAX_FREQ - MIN_FREQ)


// freq hopping
uint16_t channels[] = 
{
    MIN_FREQ, 
    MAX_FREQ, 
    MIN_FREQ + FREQ_RANGE * 1 / 2,
    MIN_FREQ + FREQ_RANGE * 1 / 4, 
    MIN_FREQ + FREQ_RANGE * 3 / 4,
    MIN_FREQ + FREQ_RANGE * 1 / 8, 
    MIN_FREQ + FREQ_RANGE * 7 / 8, 
    MIN_FREQ + FREQ_RANGE * 3 / 8,
    MIN_FREQ + FREQ_RANGE * 5 / 8, 
};

// single freq
// uint16_t channels[] = 
// {
//     MIN_FREQ + FREQ_RANGE * 3 / 4
// };


#define TOTAL_CHANNELS (sizeof(channels) / sizeof(uint16_t))

//#define RADIO_CHANNEL 96

// scan for synchronous code
#define FIFORSTREG 0xCA81
// read continuously
//#define FIFORSTREG              (0xCA81 | 0x0004)

// 915MHz
#define FREQ_BAND 0x0030

// Center Frequency: 915.000MHz
//#define CFSREG (0xA000 | RADIO_CHANNEL)
#define CFSREG(chan) (0xA000 | (chan))
// crystal load 10pF
#define XTAL_LD_CAP 0x0003
// power management page 16
#define PMCREG 0x8201
// config page 16
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

//#define TXCREG 0x9850     // FSK shift: 90kHz Full TX power
//#define TXCREG 0x98f0       // FSK shift: 165kHz TX power: 0db
#define TXCREG 0x98f2       // FSK shift: 165kHz TX power: -5db for RFX1010
//#define TXCREG 0x98f4       // FSK shift: 165kHz TX power: -10db for RFX1010
//#define TXCREG 0x98f7       // FSK shift: 165kHz TX power: -17.5db for RFX1010
#define STSREG 0x0000
#define RXFIFOREG 0xb000

// analog filter for raw mode
#define BBFCREG                 0xc23c


// audio
// speaker as peaks at -1876 & -938
// TODO: shift major scale to -1876 to -938
#define MIN_NOTE -2500
#define MAX_NOTE -1250
#define OCTAVE0 (MIN_NOTE - MIN_NOTE * 2)
#define OCTAVE1 (MAX_NOTE - MIN_NOTE)
#define OCTAVE2 (MAX_NOTE / 2 - MAX_NOTE)

// index to freq in CPU clocks
const uint16_t freqs[] = 
{
    MIN_NOTE * 2, // C0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.122465), // D0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.259933), // E0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.334849), // F0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.498309), // G0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.681796), // A0
    (uint16_t)(MIN_NOTE * 2 + OCTAVE0 * 0.887759), // B0
    MIN_NOTE,  // C1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.122465), // D1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.259933), // E1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.334849), // F1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.498309), // G1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.681796), // A1
    (uint16_t)(MIN_NOTE + OCTAVE1 * 0.887759), // B1
    MAX_NOTE, // C2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.122465), // D2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.259933), // E2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.334849), // F2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.498309), // G2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.681796), // A2
    (uint16_t)(MAX_NOTE + OCTAVE2 * 0.887759), // B2
    MAX_NOTE / 2 // C3
};

// indexes for different notes
#define _C0 0
#define _D0 1
#define _E0 2
#define _F0 3
#define _G0 4
#define _A0 5
#define _B0 6
#define _C1 7
#define _D1 8
#define _E1 9
#define _F1 10
#define _G1 11
#define _A1 12
#define _B1 13
#define _C2 14
#define _D2 15
#define _E2 16
#define _F2 17
#define _G2 18
#define _A2 19
#define _B2 20
#define _C3 21
#define SONG_REST 0xfe
#define SONG_END 0xff

typedef struct
{
// ticks before next note
    uint8_t delay;   
// note index
    uint8_t freq_index;
} song_t;



typedef union 
{
	struct
	{
		unsigned interrupt_complete : 1;
        unsigned have_stick : 1;
        unsigned send_packet : 1;
        unsigned playing_sound : 1;
        unsigned increase_pressed : 1;
        unsigned decrease_pressed : 1;
        unsigned disable_adc : 1;
	};
	
	unsigned char value;
} flags_t;

#define DURATION 1
#define DURATION2 4
const song_t increase_tone[] =
{
    { DURATION, _G1 },
    { DURATION, _A1 },
    { DURATION, _B1 },
    { DURATION, _C2 },
    { DURATION, _D2 },
    { DURATION, _E2 },
    { DURATION, _F2 },
    { DURATION, _G2 },
    { 0, SONG_END },
};


const song_t decrease_tone[] =
{
    { DURATION, _G2 },
    { DURATION, _F2 },
    { DURATION, _E2 },
    { DURATION, _D2 },
    { DURATION, _C2 },
    { DURATION, _B1 },
    { DURATION, _A1 },
    { DURATION, _G1 },
    { 0, SONG_END },
};

const song_t flat_tone[] =
{
    { DURATION, _G1 },
    { DURATION, _G2 },
    { DURATION, _G1 },
    { DURATION, _G2 },
    { DURATION, _G1 },
    { DURATION, _G2 },
    { DURATION, _G1 },
    { DURATION, _G2 },
    { 0, SONG_END },
};

#define MAX_SONG 32
song_t song_buffer[MAX_SONG];

flags_t flags;
uint32_t tick = 0;
// add fake glitches for testing
uint8_t glitch_counter = 0;
uint16_t song_tick = 0;
uint8_t song_offset = 0;
const song_t *current_song = 0;
uint8_t current_channel = 0;
uint16_t current_note = 0;
uint16_t current_delay = 0;
uint8_t increase_debounce = 0;
uint8_t decrease_debounce = 0;
#define DEBOUNCE_THRESHOLD (HZ / 10 + 1)


// hall effect sensors
uint32_t steering_accum = 0;
uint32_t steering_count = 0;
uint32_t throttle_accum = 0;
uint32_t throttle_count = 0;
uint8_t adc_watchdog = 0;
#define ADC_TIMEOUT HZ
int8_t speed_offset = 0;
// limited by song length
#define MIN_SPEED_OFFSET -8
#define MAX_SPEED_OFFSET 8

// values for bottom half
uint32_t steering_accum2 = 0;
uint32_t steering_count2 = 0;
uint32_t throttle_accum2 = 0;
uint32_t throttle_count2 = 0;
int8_t speed_offset2 = 0;






void write_radio(uint16_t data)
{
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

    current_channel++;
    if(current_channel >= TOTAL_CHANNELS)
    {
        current_channel = 0;
    }
    
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
// turn on the transmitter to tune
    write_radio(PMCREG | 0x0020);
//    write_radio(PMCREG | 0x0038);

// warm up.  1:4 prescaler for 32Mhz clock
//    LED_LAT = 0;
    T1CON = 0b10100001;
    TMR1 = RADIO_DELAY;
    PIR1bits.TMR1IF = 0;
    while(!PIR1bits.TMR1IF)
    {
    }
    T1CON = 0b10000000;
// DEBUG
//LED_LAT = 1;

// can't use ADC & amplifier
    flags.disable_adc = 1;
// turn on amplifier
    AMP_TRIS = 0;
    AMP_LAT = 1;
    ClrWdt();
}

void radio_off()
{
// turn off amplifier
    AMP_TRIS = 0;
    AMP_LAT = 0;
    flags.disable_adc = 0;
    ADCON0bits.GO = 1;
// DEBUG
//LED_LAT = 0;

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
    while(!PIR1bits.TXIF)
    {
    }

    TXREG = value;    
}


void play_song(const song_t *song)
{
    song_tick = 0;
    song_offset = 0;
    flags.playing_sound = 1;
    current_song = song;
    current_note = freqs[song[0].freq_index];
    current_delay = song[0].delay;
    T3CON = 0b10000001;
    TMR3 = current_note;
    PIR2bits.TMR3IF = 0;
    PIE2bits.TMR3IE = 1;
    SPEAKER_TRIS1 = 0;
    SPEAKER_TRIS2 = 0;
    SPEAKER_LAT1 = 1;
    SPEAKER_LAT2 = 0;
}

void stop_song()
{
    flags.playing_sound = 0;
    T3CON = 0b10000000;
    PIE2bits.TMR3IE = 0;
    PIR2bits.TMR3IF = 0;
    SPEAKER_TRIS1 = 1;
    SPEAKER_TRIS2 = 1;
    SPEAKER_LAT1 = 0;
    SPEAKER_LAT2 = 0;
}


uint8_t copy_song(const song_t *src)
{
    uint8_t i = 0;
    while(1)
    {
        song_buffer[i] = src[i];
        if(src[i].freq_index == SONG_END)
        {
            i++;
            break;
        }
        i++;
    }
    
    return i;
}

void play_current_speed()
{
    if(speed_offset > 0)
    {
        uint8_t i = copy_song(increase_tone);
        uint8_t j;
// overwrite the end code
        i--;
        for(j = 0; j < speed_offset && i <= MAX_SONG - 3; j++)
        {
            song_buffer[i].delay = HZ * 1 / 5;
            song_buffer[i].freq_index = SONG_REST;
//            song_buffer[i].freq_index = _C0;
            i++;
            song_buffer[i].delay = HZ * 1 / 5;
            song_buffer[i].freq_index = _G2;
            i++;
        }
        song_buffer[i].delay = 0;
        song_buffer[i].freq_index = SONG_END;
    }
    else
    if(speed_offset < 0)
    {
        uint8_t i = copy_song(decrease_tone);
        uint8_t j;
// overwrite the end code
        i--;
        for(j = 0; j < -speed_offset && i <= MAX_SONG - 3; j++)
        {
            song_buffer[i].delay = HZ * 1 / 5;
            song_buffer[i].freq_index = SONG_REST;
//            song_buffer[i].freq_index = _C1;
            i++;
            song_buffer[i].delay = HZ * 1 / 5;
            song_buffer[i].freq_index = _G1;
            i++;
        }

        song_buffer[i].delay = 0;
        song_buffer[i].freq_index = SONG_END;
    }
    else
    {
        copy_song(flat_tone);
    }

    play_song(song_buffer);
}

void write_speed_offset()
{
    EEADR = 0;
    EEDATA = speed_offset;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;

    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xaa;
    EECON1bits.WR = 1;
    INTCONbits.GIE = 1;
    
    EECON1bits.WREN = 0;
}

void read_speed_offset()
{
    EEADR = 0;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    speed_offset = EEDATA;
    if(speed_offset > MAX_SPEED_OFFSET ||
        speed_offset < MIN_SPEED_OFFSET)
    {
        speed_offset = 0;
    }
}

void handle_speed_increase()
{
    speed_offset++;
    if(speed_offset > MAX_SPEED_OFFSET)
    {
        speed_offset = MAX_SPEED_OFFSET;
    }
    
    play_current_speed();
    write_speed_offset();
}

void handle_speed_decrease()
{
    speed_offset--;
    if(speed_offset < MIN_SPEED_OFFSET)
    {
        speed_offset = MIN_SPEED_OFFSET;
    }
    
    play_current_speed();
    write_speed_offset();
}


#define DEBOUNCE(flag, pin, counter, handler) \
{ \
    if(flag) \
    { \
/* button released */ \
        if(pin) \
        { \
            counter++; \
            if(counter >= DEBOUNCE_THRESHOLD) \
            { \
                flag = 0; \
                counter = 0; \
            } \
        } \
        else \
        { \
            counter = 0; \
        } \
    } \
    else \
    { \
        if(!pin) \
        { \
/* button pressed */ \
            counter++; \
            if(counter >= DEBOUNCE_THRESHOLD) \
            { \
                flag = 1; \
                counter = 0; \
                handler;  \
            } \
        } \
        else \
        { \
            counter = 0; \
        } \
    } \
}

void main()
{
    OSCCON = 0b11100000;


// LED on
    LED_LAT = 1;
    LED_TRIS = 0;
    SPEAKER_TRIS1 = 0;
    SPEAKER_TRIS2 = 0;

//LATBbits.LATB5 = 0;
//TRISBbits.TRISB5 = 0;

// turn off amplifier
    AMP_TRIS = 0;
    AMP_LAT = 0;

	flags.value = 0;
    tick = 0;


// start ADC
    ANSEL = 0b00110000;
    ANSELH = 0b00000000;
    ADCON0 = 0b00010001;
// the RC is the only reliable conversion clock when using the amplifier
    ADCON2 = 0b10111111;
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;

    radio_off();
    serial_on();

    read_speed_offset();

// mane timer
// 1:32 prescaler for 32Mhz clock
    T0CON = 0b10000100;
    TMR0 = TIMER0_VALUE;


//     uint8_t init_delay = 0;
//     while(init_delay < HZ * 2)
//     {
//         if(INTCONbits.TMR0IF)
//         {
//             ClrWdt();
//             TMR0 = TIMER0_VALUE;
//             INTCONbits.TMR0IF = 0;
//             init_delay++;
//         }
//     }



    INTCON = 0b11100000;

// start getting stick values
    ADCON0bits.GO = 1;

    while(1)
    {
//        ClrWdt();
// DEBUG
//LED_LAT = !LED_LAT;

        if(flags.send_packet)
        {
            LED_LAT = !LED_LAT;

// DEBUG
// #define FAKE_GLITCHES
// #ifdef FAKE_GLITCHES
// glitch_counter++;
// if(glitch_counter >= TOTAL_CHANNELS) glitch_counter = 0;
// #endif

// transmit packet
            flags.send_packet = 0;

            uint8_t steering_value = steering_accum2 / steering_count2 / 4;
            uint8_t throttle_value = throttle_accum2 / throttle_count2 / 4;

            radio_on();
// delay for amplifier & framing errors
            write_serial(0xff);
            write_serial(0xff);
            write_serial(0xff);
            write_serial(0xff);

            uint8_t i;
            for(i = 0; i < sizeof(PACKET_KEY); i++)
            {
                write_serial(PACKET_KEY[i]);
// DEBUG
// #ifdef FAKE_GLITCHES
// if(!glitch_counter && i == 4)
// {
//     write_serial(0x0);
// }
// #endif
            }


            write_serial(steering_value ^ DATA_KEY[0]);
            write_serial(throttle_value ^ DATA_KEY[1]);
            write_serial(speed_offset2 ^ DATA_KEY[2]);

            write_serial(steering_value ^ DATA_KEY[3]);
            write_serial(throttle_value ^ DATA_KEY[4]);
            write_serial(speed_offset2 ^ DATA_KEY[5]);

// DEBUG
// #ifdef FAKE_GLITCHES
// if(glitch_counter < 2)
// {
//     write_serial(0x0);
// }
// #endif
            write_serial(steering_value ^ DATA_KEY[6]);
            write_serial(throttle_value ^ DATA_KEY[7]);
            write_serial(speed_offset2 ^ DATA_KEY[8]);

            write_serial(steering_value ^ DATA_KEY[9]);
            write_serial(throttle_value ^ DATA_KEY[10]);
            write_serial(speed_offset2 ^ DATA_KEY[11]);

// DEBUG
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);
//                 write_serial(tick);
//                 write_serial(blink_counter);

            flush_serial();
            radio_off();
        }
    }
    
}

void __interrupt(low_priority) isr1()
{
}

void __interrupt(high_priority) isr()
{
    flags.interrupt_complete = 0;
// DEBUG
//LED_LAT = !LED_LAT;
	while(!flags.interrupt_complete)
	{
		flags.interrupt_complete = 1;

// tick counter
		if(INTCONbits.TMR0IF)
		{
// DEBUG
//LED_LAT = !LED_LAT;
			INTCONbits.TMR0IF = 0;
            TMR0 = TIMER0_VALUE;
			flags.interrupt_complete = 0;
            tick++;
            adc_watchdog++;

            if(adc_watchdog > ADC_TIMEOUT &&
                (steering_count == 0 ||
                throttle_count == 0))
            {
                Reset();
            }

// send packet
            if(flags.have_stick &&
                steering_count > 0 &&
                throttle_count > 0)
            {
// cue the bottom half
                flags.send_packet = 1;

// don't do 32 bit math in the interrupt handler
                steering_accum2 = steering_accum;
                throttle_accum2 = throttle_accum;
                steering_count2 = steering_count;
                throttle_count2 = throttle_count;
                steering_accum = 0;
                throttle_accum = 0;
                steering_count = 0;
                throttle_count = 0;
                speed_offset2 = speed_offset;
            }

// test buttons only when not transmitting
//            if(!flags.disable_adc)
//            {
                DEBOUNCE(flags.increase_pressed, \
                    INCREASE_PORT, \
                    increase_debounce, \
                    handle_speed_increase())
                DEBOUNCE(flags.decrease_pressed, \
                    DECREASE_PORT, \
                    decrease_debounce, \
                    handle_speed_decrease())
//            }

// advance song
            if(flags.playing_sound)
            {
                song_tick++;
                if(song_tick >= current_delay)
                {
                    song_offset++;
                    song_t *ptr = &song_buffer[song_offset];
                    if(ptr->freq_index == SONG_END)
                    {
                        stop_song();
                    }
                    else
                    {
                        song_tick = 0;
                        current_delay = ptr->delay;
                        if(ptr->freq_index == SONG_REST)
                        {
                            SPEAKER_TRIS1 = 1;
                            SPEAKER_TRIS2 = 1;
                        }
                        else
                        {
                            current_note = freqs[ptr->freq_index];
                            PIR2bits.TMR3IF = 0;
                            PIE2bits.TMR3IE = 1;
                            TMR3 = current_note;
                            SPEAKER_TRIS1 = 0;
                            SPEAKER_TRIS2 = 0;
                        }
                    }
                }
            }
        }


// ADC
        if(PIR1bits.ADIF)
        {
// DEBUG
//LED_LAT = !LED_LAT;
            flags.interrupt_complete = 0;
            PIR1bits.ADIF = 0;
            adc_watchdog = 0;
            
            if(!flags.disable_adc)
            {

                if(ADCON0 == 0b00010001)
                {
                    ADCON0 = 0b00010101;
                    steering_accum += ADRES;
                    steering_count++;
                }
                else
                {
                    ADCON0 = 0b00010001;
                    throttle_accum += ADRES;
                    throttle_count++;
                }
                ADCON0bits.GO = 1;

    // capture real data after warming up
                if(!flags.have_stick &&
                    tick >= HZ / 5)
                {
                    throttle_accum = 0;
                    throttle_count = 0;
                    steering_accum = 0;
                    steering_count = 0;
                    flags.have_stick = 1;
                    play_current_speed();
                }
            }
        }

// sound
        if(PIR2bits.TMR3IF)
        {
            flags.interrupt_complete = 0;
            PIR2bits.TMR3IF = 0;
            
            if(flags.playing_sound)
            {
                TMR3 = current_note;
                SPEAKER_LAT1 = !SPEAKER_LAT1;
                SPEAKER_LAT2 = !SPEAKER_LAT2;
            }
        }
    }
}










