/*
 * REMOTE CONTROL FOR CAM PANNER
 * Copyright (C) 2020-2023 Adam Williams <broadcast at earthling dot net>
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
#pragma config WDTEN = OFF       // Watchdog Timer Enable bit
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config HFOFST = OFF     // HFINTOSC Fast Start-up bit (The system clock is held off until the HFINTOSC is stable.)

// CONFIG4L
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)




// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// maximum speed for sounds
#define CLOCKSPEED 32000000
// steering sounds
#define USE_STEERING_SOUND

// analog values from arm_cam.c
// an offset is added to the ADC to get this range
#define ADC_CENTER 128
#define ADC_DEADBAND 5
// max offset for computing a center value, otherwise
// we load it from flash
#define MAX_ADC_OFFSET 32
// minimums
#define MIN_LEFT (ADC_CENTER + ADC_DEADBAND)
#define MIN_RIGHT (ADC_CENTER - ADC_DEADBAND)
// maximums
#define MAX_LEFT (ADC_CENTER + MAX_ADC_OFFSET)
#define MAX_RIGHT (ADC_CENTER - MAX_ADC_OFFSET)

// the system clock & the number of packets per second
#define HZ 25
// samples per packet
#define ADC_PER_PACKET 64

// delay between packets
#define TIMER0_VALUE (-CLOCKSPEED / 4 / 32 / HZ)



const uint8_t PACKET_KEY[] = 
{
    0x5e, 0x1b, 0xdb, 0xc8, 0x98, 0xa1, 0x5e, 0x90
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0x00, 0x00, 0xaa, 0xaa, 0x55, 0x55
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
// freq hopping.  Offset the traction controller frequencies.
const uint16_t channels[] = 
{
    MIN_FREQ + FREQ_RANGE / 16, 
    MAX_FREQ - FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 1 / 2 + FREQ_RANGE / 16 - 100, // fudge the spacing
    MIN_FREQ + FREQ_RANGE * 1 / 4 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 3 / 4 - FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 1 / 8 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 7 / 8 - FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 3 / 8 + FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 5 / 8 - FREQ_RANGE / 16, 
};

// single freq for testing
// uint16_t channels[] = 
// {
//     MIN_FREQ + FREQ_RANGE * 1 / 2
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
// index to freq in CPU clocks
#define LATENCY 0
// tuned based on speaker & clockspeed
#define BASE_FREQ 5886628
//#define BASE_FREQ 3000000
#define FREQ_TO_PERIOD(f) (uint16_t)(-BASE_FREQ / 4 / (f * 2) + LATENCY)


const uint16_t freqs[] = 
{
	FREQ_TO_PERIOD(130.81), // C0
	FREQ_TO_PERIOD(138.59), // _Db0
	FREQ_TO_PERIOD(146.83), // _D0
	FREQ_TO_PERIOD(155.56), // _Eb0
	FREQ_TO_PERIOD(164.81), // _E0
	FREQ_TO_PERIOD(174.61), // _F0
	FREQ_TO_PERIOD(185.00), // _Gb0
	FREQ_TO_PERIOD(196.00), // _G0
	FREQ_TO_PERIOD(207.65), // _Ab0
	FREQ_TO_PERIOD(220.00), // _A0
	FREQ_TO_PERIOD(233.08), // _Bb0
	FREQ_TO_PERIOD(246.94), // _B0

	FREQ_TO_PERIOD(261.63), // C1
	FREQ_TO_PERIOD(277.18), 
	FREQ_TO_PERIOD(293.66), 
	FREQ_TO_PERIOD(311.13), 
	FREQ_TO_PERIOD(329.63), 
	FREQ_TO_PERIOD(349.23), 
	FREQ_TO_PERIOD(369.99), 
	FREQ_TO_PERIOD(392.00), // G1
	FREQ_TO_PERIOD(415.30), 
	FREQ_TO_PERIOD(440.00), 
	FREQ_TO_PERIOD(466.16), 
	FREQ_TO_PERIOD(493.88), 

	FREQ_TO_PERIOD(523.251), // C2
	FREQ_TO_PERIOD(554.365),
	FREQ_TO_PERIOD(587.330),
	FREQ_TO_PERIOD(622.254),
	FREQ_TO_PERIOD(659.255),
	FREQ_TO_PERIOD(698.456),
	FREQ_TO_PERIOD(739.989),
	FREQ_TO_PERIOD(783.991),
	FREQ_TO_PERIOD(830.609),
	FREQ_TO_PERIOD(880.000),
	FREQ_TO_PERIOD(932.328),
	FREQ_TO_PERIOD(987.767),

	FREQ_TO_PERIOD(1046.50), // C3
	FREQ_TO_PERIOD(1108.73),
	FREQ_TO_PERIOD(1174.66),
	FREQ_TO_PERIOD(1244.51),
	FREQ_TO_PERIOD(1318.51),
	FREQ_TO_PERIOD(1396.91),
	FREQ_TO_PERIOD(1479.98),
	FREQ_TO_PERIOD(1567.98),
	FREQ_TO_PERIOD(1661.22),
	FREQ_TO_PERIOD(1760.00),
	FREQ_TO_PERIOD(1864.66),
	FREQ_TO_PERIOD(1975.53),

	FREQ_TO_PERIOD(2093.00) // C4
};

// indexes for different notes
#define _C0 0
#define _Db0 1
#define _D0 2
#define _Eb0 3
#define _E0 4
#define _F0 5
#define _Gb0 6
#define _G0 7
#define _Ab0 8
#define _A0 9
#define _Bb0 10
#define _B0 11

#define _C1 12
#define _Db1 13
#define _D1 14
#define _Eb1 15
#define _E1 16
#define _F1 17
#define _Gb1 18
#define _G1 19
#define _Ab1 20
#define _A1 21
#define _Bb1 22
#define _B1 23

#define _C2 24
#define _Db2 25
#define _D2 26
#define _Eb2 27
#define _E2 28
#define _F2 29
#define _Gb2 30
#define _G2 31
#define _Ab2 32
#define _A2 33
#define _Bb2 34
#define _B2 35

#define _C3 36
#define _Db3 37
#define _D3 38
#define _Eb3 39
#define _E3 40
#define _F3 41
#define _Gb3 42
#define _G3 43
#define _Ab3 44
#define _A3 45
#define _Bb3 46
#define _B3 47

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
        unsigned send_packet : 1;
        unsigned have_stick : 1;
        unsigned playing_sound : 1;
#ifdef USE_STEERING_SOUND
        unsigned is_steering : 1;
#endif
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

const song_t steering_sound[] =
{
    { DURATION * 1, _C2 },
    { DURATION * 3, SONG_REST },
    { DURATION * 1, _B1 },
    { DURATION * 3, SONG_REST },
    { 0, SONG_END }
};

#define MAX_SONG 32
song_t song_buffer[MAX_SONG];

#define TIMELAPSE_OFF 0x3
#define TIMELAPSE_LEFT 0x0
#define TIMELAPSE_RIGHT 0x1

#define LED_TICKS (HZ / 5)

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
uint8_t timelapse_mode = TIMELAPSE_OFF;
uint8_t blink_offset = 0;
uint8_t blink_counter = 0;
// hall effect accumulator
uint32_t adc_accum;
uint32_t adc_count;
// computed at startup
int8_t adc_offset = 0;
uint8_t current_channel = 0;
uint16_t song_tick = 0;
uint8_t song_offset = 0;
uint16_t current_note = 0;
uint16_t current_delay = 0;

void get_timelapse();
void wait_timelapse();
void (*adc_state)() = get_timelapse;

// called every tick
void handle_led()
{
    if(adc_state == get_timelapse)
    {
// do nothing until timelapse mode is known
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
// enable serial port 10mA
    TXSTA = 0b00100100;
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

// turn on amplifier
    AMP_TRIS = 0;
    AMP_LAT = 1;
}

void radio_off()
{
// disable serial port 10mA
    TXSTA = 0b00000100;

// turn off amplifier
    AMP_TRIS = 0;
    AMP_LAT = 0;
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

void write_serial(uint8_t value)
{
    while(!PIR1bits.TXIF)
    {
    }

    TXREG = value;    
}



// have to copy the sound in order to add the tones for the current speed
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

void play_song()
{
    song_tick = 0;
    song_offset = 0;
    flags.playing_sound = 1;
    current_note = freqs[song_buffer[0].freq_index];
    current_delay = song_buffer[0].delay;
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
// turn off speaker timer
    T3CON = 0b10000000;
    PIE2bits.TMR3IE = 0;
    PIR2bits.TMR3IF = 0;
    SPEAKER_TRIS1 = 1;
    SPEAKER_TRIS2 = 1;
    SPEAKER_LAT1 = 0;
    SPEAKER_LAT2 = 0;
}

void get_stick()
{
    if(adc_count < ADC_PER_PACKET) ADCON0bits.GO = 1;
}

// wait for stick to be released to prevent timelapse selection from moving motor
void wait_timelapse()
{
// fall through to print ADC value on receiver
    const int8_t bypass = 0;
    if(adc_count >= 64)
    {
        uint8_t value = adc_accum / adc_count / 4;
        if(!bypass) value -= adc_offset;

// centered
        if(bypass || 
            (value < MIN_LEFT &&
            value >= MIN_RIGHT))
        {
            adc_state = get_stick;
// move to manual mode
            flags.have_stick = 1;
        }
        
        adc_accum = 0;
        adc_count = 0;
    }
    ADCON0bits.GO = 1;
}


void write_byte(uint8_t address, uint8_t value)
{
    EEADR = address;
    EEDATA = value;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN = 1;

    INTCONbits.GIE = 0;
    EECON2 = 0x55;
    EECON2 = 0xaa;
    EECON1bits.WR = 1;
// wait for write
    while(EECON1bits.WR) 
        ;
    
    INTCONbits.GIE = 1;
    EECON1bits.WREN = 0;
}

void save_settings()
{
    write_byte(0, adc_offset);
}

uint8_t read_byte(uint8_t address)
{
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.CFGS = 0;
    EECON1bits.RD = 1;
    return EEDATA;
}

void load_settings()
{
    adc_offset = read_byte(0);
}

// get timelapse code
void get_timelapse()
{
    if(adc_count >= 64)
    {
        uint8_t value = adc_accum / adc_count / 4;
// get new center offset
        if(value >= ADC_CENTER - MAX_ADC_OFFSET &&
            value < ADC_CENTER + MAX_ADC_OFFSET)
        {
            adc_offset = value - ADC_CENTER;
            save_settings();
        }
        else
        {
// load previous center offset
            load_settings();
        }

// center it
        value -= adc_offset;

        if(value <= MAX_RIGHT)
        {
            timelapse_mode = TIMELAPSE_RIGHT;
            copy_song(decrease_tone);
        }
        else
        if(value >= MAX_LEFT)
        {
            timelapse_mode = TIMELAPSE_LEFT;
            copy_song(increase_tone);
        }
        else
        {
            copy_song(flat_tone);
        }
        play_song();

        adc_accum = 0;
        adc_count = 0;
        adc_state = wait_timelapse;


// start the LED for the timelapse code
        const uint8_t *blink_pattern = blink_patterns + 
            timelapse_mode * BLINK_PATTERN_SIZE;
        blink_counter = blink_pattern[0];
        LED_LAT = 0;
        tick = 0;
    }
    ADCON0bits.GO = 1;
}


void main()
{
// 32 Mhz
    OSCCON = 0b11100000;


// LED off until initial button state is captured
    LED_LAT = 1;
    LED_TRIS = 0;
// turn off amplifier
    AMP_TRIS = 0;
    AMP_LAT = 0;
    SPEAKER_TRIS1 = 1;
    SPEAKER_TRIS2 = 1;


    flags.value = 0;
    tick = 0;


// start ADC
// AN4 is the hall effect sensor
    ANSEL = 0b00010000;
    ANSELH = 0b00000000;
    ADCON0 = 0b00010001;
// ADC clock is Fosc/64
    ADCON2 = 0b10111110;
    PIR1bits.ADIF = 0;
// off during initialization
    LED_LAT = 1;

    radio_off();
    serial_on();


// mane timer
// 1:32 prescaler for 32Mhz clock
    T0CON = 0b10000100;
    TMR0 = TIMER0_VALUE;

// start getting stick values
    ADCON0bits.GO = 1;

    uint8_t init_delay = 0;
    while(init_delay < HZ / 4)
    {
        if(INTCONbits.TMR0IF)
        {
            TMR0 = TIMER0_VALUE;
            INTCONbits.TMR0IF = 0;
            init_delay++;
        }
    }

    INTCON = 0b11100000;




    uint8_t adc_value = 0;
    while(1)
    {
        if(flags.send_packet)
        {
            flags.send_packet = 0;
            handle_led();

            if(flags.have_stick)
            {
// DEBUG
//LED_LAT = 0;
// transmit packet
                radio_on();

// delay for amplifier & framing errors
                write_serial(0xff);
                write_serial(0xff);
                write_serial(0xff);
                write_serial(0xff);

                adc_value = adc_accum / adc_count / 4;
                adc_value -= adc_offset;

                uint8_t i;
                for(i = 0; i < sizeof(PACKET_KEY); i++)
                {
                    write_serial(PACKET_KEY[i]);
                }
                write_serial(timelapse_mode ^ DATA_KEY[0]);
                write_serial(adc_value ^ DATA_KEY[1]);

                write_serial(timelapse_mode ^ DATA_KEY[2]);
                write_serial(adc_value ^ DATA_KEY[3]);

                write_serial(timelapse_mode ^ DATA_KEY[4]);
                write_serial(adc_value ^ DATA_KEY[5]);

                write_serial(timelapse_mode ^ DATA_KEY[6]);
                write_serial(adc_value ^ DATA_KEY[7]);

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
// DEBUG
//LED_LAT = 1;

            }

// advance song with or without stick & before testing steering
            if(flags.playing_sound)
            {
                song_tick++;
                if(song_tick >= current_delay)
                {
                    song_offset++;
                    song_t *ptr = &song_buffer[song_offset];
                    if(ptr->freq_index == SONG_END)
                    {
    #ifdef USE_STEERING_SOUND
    // loop the steering sound
                        if(flags.is_steering)
                        {
                            copy_song(steering_sound);
                            play_song();
                        }
                        else
    #endif
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

            if(flags.have_stick)
            {
    #ifdef USE_STEERING_SOUND
    // start song after advancing song
                uint8_t steering2 = adc_value;
    // play steering sound as long as it's steering
                if(!flags.playing_sound && 
                    ((steering2 > MIN_LEFT) ||
                    (steering2 < MIN_RIGHT)))
                {
                    copy_song(steering_sound);
                    play_song();
                    flags.is_steering = 1;
                }
                else
                if(flags.is_steering && 
                    steering2 >= MIN_RIGHT &&
                    steering2 <= MIN_LEFT)
                {
                    flags.is_steering = 0;
                    stop_song();
                }
    #endif // USE_STEERING_SOUND


    // start a new set of ADC
                adc_accum = 0;
                adc_count = 0;
                ADCON0bits.GO = 1;
            }


        }




// ADC
        if(PIR1bits.ADIF)
        {
            PIR1bits.ADIF = 0;

            adc_accum += ADRES;
            adc_count++;
// DEBUG
//LED_LAT = !LED_LAT;

            adc_state();
        }


    }
    
}

void interrupt isr()
{
	while(1)
	{
		flags.interrupt_complete = 1;

// tick counter
		if(INTCONbits.TMR0IF)
		{
			INTCONbits.TMR0IF = 0;
            TMR0 = TIMER0_VALUE;
			flags.interrupt_complete = 0;
            tick++;

// cue the bottom half
            flags.send_packet = 1;
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

		if(flags.interrupt_complete) break;
    }
}










