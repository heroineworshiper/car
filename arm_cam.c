/*
 * STM32 CONTROLLER FOR CAMERA PANNER
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

// Useful commands to install it:

// make cam.bin;./uart_programmer cam.bin




#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"



// settings
// time
#define HZ 1000
#define PWM_PERIOD 409
// 0 - PWM_PERIOD
#define POWER (PWM_PERIOD)

// speed depends on the number of poles in the motor & the gear ratio
#define POLES 8
// 4.5 gear ratio hobby motor
//#define SCALE_FACTOR (4.5 * POLES * FRACTION)
// servo
#define SCALE_FACTOR (40 * POLES * FRACTION)

// time before shutting motor down
#define MOTOR_TIMEOUT (HZ / 4)

// Normally want to reach target speed in 1 second
// degrees per second per second.  
//#define SLOW_ACCELERATION (90 * SCALE_FACTOR)
//#define FAST_ACCELERATION (90 * SCALE_FACTOR)
#define ACCELERATION (90 * SCALE_FACTOR)
// degrees per second
#define FAST_SPEED (90 * SCALE_FACTOR)
#define SLOW_SPEED (10 * SCALE_FACTOR)


// timelapse is normally 20:1 to match running bounces & 
// we want it to look like 1/20 of normal speeds
// degrees per second
#define SLOW_TIMELAPSE (SLOW_SPEED / 80)
#define FAST_TIMELAPSE (FAST_SPEED / 80)


// analog ranges
// deadband
#define MIN_LEFT 130
#define MIN_RIGHT 126
// maximums
#define MAX_LEFT 240
#define MAX_RIGHT 20

// speed updates/second
// hobby motor
//#define FRAME_HZ 100
// servo
#define FRAME_HZ 1000


//#define DEBUG_PIN GPIO_Pin_4
//#define DEBUG_GPIO GPIOB

#define LED_PIN GPIO_Pin_0
#define LED_GPIO GPIOB

#define ENABLE_PIN GPIO_Pin_7
#define ENABLE_GPIO GPIOA


// frequency hopping rate
#define HOP_HZ 25
// rate when scanning
#define SCAN_HZ 5
// time before next packet we should hop (10ms)
#define HOP_LAG (HZ / 100)

const uint8_t PACKET_KEY[] = 
{
    0x5e, 0x1b, 0xdb, 0xc8, 0x98, 0xa1, 0x5e, 0x90
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0x00, 0x00, 0xaa, 0xaa, 0x55, 0x55
};

#define PACKET_DATA 8

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
    MIN_FREQ + FREQ_RANGE * 1 / 2 + FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 1 / 4 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 3 / 4 - FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 1 / 8 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 7 / 8 - FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 3 / 8 + FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 5 / 8 - FREQ_RANGE / 16, 
};

#define TOTAL_CHANNELS (sizeof(channels) / sizeof(uint16_t))



// RADIO_CHANNEL is from 96-3903 & set by the user
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

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

volatile int current_channel = 0;
volatile int missed_packets = 0;
// start time of last received packet
volatile int packet_tick = 0;
// time of next hop
volatile int next_hop = 0;
volatile int need_hop = 0;
volatile int scanning = 0;
#define MAX_MISSED_PACKETS HOP_HZ







const uint16_t sin_table[] = {
    0x7fff, 0x80c8, 0x8191, 0x825a, 0x8323, 0x83ec, 0x84b5, 0x857e, 0x8647, 0x8710, 0x87d8, 0x88a1, 0x896a, 0x8a32, 0x8afa, 0x8bc3, 
    0x8c8b, 0x8d53, 0x8e1b, 0x8ee2, 0x8faa, 0x9072, 0x9139, 0x9200, 0x92c7, 0x938e, 0x9454, 0x951b, 0x95e1, 0x96a7, 0x976d, 0x9832, 
    0x98f8, 0x99bd, 0x9a82, 0x9b46, 0x9c0a, 0x9cce, 0x9d92, 0x9e56, 0x9f19, 0x9fdc, 0xa09e, 0xa161, 0xa223, 0xa2e4, 0xa3a5, 0xa466, 
    0xa527, 0xa5e7, 0xa6a7, 0xa766, 0xa826, 0xa8e4, 0xa9a3, 0xaa61, 0xab1e, 0xabdb, 0xac98, 0xad54, 0xae10, 0xaecb, 0xaf86, 0xb041, 
    0xb0fb, 0xb1b4, 0xb26d, 0xb326, 0xb3de, 0xb495, 0xb54c, 0xb603, 0xb6b9, 0xb76e, 0xb823, 0xb8d8, 0xb98c, 0xba3f, 0xbaf2, 0xbba4, 
    0xbc55, 0xbd07, 0xbdb7, 0xbe67, 0xbf16, 0xbfc5, 0xc073, 0xc120, 0xc1cd, 0xc279, 0xc324, 0xc3cf, 0xc47a, 0xc523, 0xc5cc, 0xc674, 
    0xc71c, 0xc7c2, 0xc869, 0xc90e, 0xc9b3, 0xca57, 0xcafa, 0xcb9d, 0xcc3f, 0xcce0, 0xcd80, 0xce20, 0xcebf, 0xcf5d, 0xcffa, 0xd097, 
    0xd132, 0xd1ce, 0xd268, 0xd301, 0xd39a, 0xd432, 0xd4c9, 0xd55f, 0xd5f4, 0xd689, 0xd71d, 0xd7af, 0xd842, 0xd8d3, 0xd963, 0xd9f3, 
    0xda81, 0xdb0f, 0xdb9c, 0xdc28, 0xdcb3, 0xdd3d, 0xddc6, 0xde4f, 0xded6, 0xdf5d, 0xdfe2, 0xe067, 0xe0eb, 0xe16e, 0xe1f0, 0xe271, 
    0xe2f1, 0xe370, 0xe3ee, 0xe46b, 0xe4e7, 0xe562, 0xe5dd, 0xe656, 0xe6ce, 0xe745, 0xe7bc, 0xe831, 0xe8a5, 0xe918, 0xe98b, 0xe9fc, 
    0xea6c, 0xeadb, 0xeb4a, 0xebb7, 0xec23, 0xec8e, 0xecf8, 0xed61, 0xedc9, 0xee2f, 0xee95, 0xeefa, 0xef5e, 0xefc0, 0xf022, 0xf082, 
    0xf0e1, 0xf140, 0xf19d, 0xf1f9, 0xf254, 0xf2ae, 0xf306, 0xf35e, 0xf3b4, 0xf40a, 0xf45e, 0xf4b1, 0xf503, 0xf554, 0xf5a4, 0xf5f3, 
    0xf640, 0xf68d, 0xf6d8, 0xf722, 0xf76b, 0xf7b3, 0xf7f9, 0xf83f, 0xf883, 0xf8c6, 0xf908, 0xf949, 0xf989, 0xf9c7, 0xfa04, 0xfa41, 
    0xfa7c, 0xfab5, 0xfaee, 0xfb25, 0xfb5c, 0xfb91, 0xfbc4, 0xfbf7, 0xfc28, 0xfc59, 0xfc88, 0xfcb6, 0xfce2, 0xfd0e, 0xfd38, 0xfd61, 
    0xfd89, 0xfdb0, 0xfdd5, 0xfdf9, 0xfe1c, 0xfe3e, 0xfe5e, 0xfe7e, 0xfe9c, 0xfeb9, 0xfed4, 0xfeef, 0xff08, 0xff20, 0xff37, 0xff4c, 
    0xff61, 0xff74, 0xff86, 0xff96, 0xffa6, 0xffb4, 0xffc1, 0xffcd, 0xffd7, 0xffe0, 0xffe8, 0xffef, 0xfff5, 0xfff9, 0xfffc, 0xfffe, 
    0xffff, 0xfffe, 0xfffc, 0xfff9, 0xfff5, 0xffef, 0xffe8, 0xffe0, 0xffd7, 0xffcd, 0xffc1, 0xffb4, 0xffa6, 0xff96, 0xff86, 0xff74, 
    0xff61, 0xff4c, 0xff37, 0xff20, 0xff08, 0xfeef, 0xfed4, 0xfeb9, 0xfe9c, 0xfe7e, 0xfe5e, 0xfe3e, 0xfe1c, 0xfdf9, 0xfdd5, 0xfdb0, 
    0xfd89, 0xfd61, 0xfd38, 0xfd0e, 0xfce2, 0xfcb6, 0xfc88, 0xfc59, 0xfc28, 0xfbf7, 0xfbc4, 0xfb91, 0xfb5c, 0xfb25, 0xfaee, 0xfab5, 
    0xfa7c, 0xfa41, 0xfa04, 0xf9c7, 0xf989, 0xf949, 0xf908, 0xf8c6, 0xf883, 0xf83f, 0xf7f9, 0xf7b3, 0xf76b, 0xf722, 0xf6d8, 0xf68d, 
    0xf640, 0xf5f3, 0xf5a4, 0xf554, 0xf503, 0xf4b1, 0xf45e, 0xf40a, 0xf3b4, 0xf35e, 0xf306, 0xf2ae, 0xf254, 0xf1f9, 0xf19d, 0xf140, 
    0xf0e1, 0xf082, 0xf022, 0xefc0, 0xef5e, 0xeefa, 0xee95, 0xee2f, 0xedc9, 0xed61, 0xecf8, 0xec8e, 0xec23, 0xebb7, 0xeb4a, 0xeadb, 
    0xea6c, 0xe9fc, 0xe98b, 0xe918, 0xe8a5, 0xe831, 0xe7bc, 0xe745, 0xe6ce, 0xe656, 0xe5dd, 0xe562, 0xe4e7, 0xe46b, 0xe3ee, 0xe370, 
    0xe2f1, 0xe271, 0xe1f0, 0xe16e, 0xe0eb, 0xe067, 0xdfe2, 0xdf5d, 0xded6, 0xde4f, 0xddc6, 0xdd3d, 0xdcb3, 0xdc28, 0xdb9c, 0xdb0f, 
    0xda81, 0xd9f3, 0xd963, 0xd8d3, 0xd842, 0xd7af, 0xd71d, 0xd689, 0xd5f4, 0xd55f, 0xd4c9, 0xd432, 0xd39a, 0xd301, 0xd268, 0xd1ce, 
    0xd132, 0xd097, 0xcffa, 0xcf5d, 0xcebf, 0xce20, 0xcd80, 0xcce0, 0xcc3f, 0xcb9d, 0xcafa, 0xca57, 0xc9b3, 0xc90e, 0xc869, 0xc7c2, 
    0xc71c, 0xc674, 0xc5cc, 0xc523, 0xc47a, 0xc3cf, 0xc324, 0xc279, 0xc1cd, 0xc120, 0xc073, 0xbfc5, 0xbf16, 0xbe67, 0xbdb7, 0xbd07, 
    0xbc55, 0xbba4, 0xbaf2, 0xba3f, 0xb98c, 0xb8d8, 0xb823, 0xb76e, 0xb6b9, 0xb603, 0xb54c, 0xb495, 0xb3de, 0xb326, 0xb26d, 0xb1b4, 
    0xb0fb, 0xb041, 0xaf86, 0xaecb, 0xae10, 0xad54, 0xac98, 0xabdb, 0xab1e, 0xaa61, 0xa9a3, 0xa8e4, 0xa826, 0xa766, 0xa6a7, 0xa5e7, 
    0xa527, 0xa466, 0xa3a5, 0xa2e4, 0xa223, 0xa161, 0xa09e, 0x9fdc, 0x9f19, 0x9e56, 0x9d92, 0x9cce, 0x9c0a, 0x9b46, 0x9a82, 0x99bd, 
    0x98f8, 0x9832, 0x976d, 0x96a7, 0x95e1, 0x951b, 0x9454, 0x938e, 0x92c7, 0x9200, 0x9139, 0x9072, 0x8faa, 0x8ee2, 0x8e1b, 0x8d53, 
    0x8c8b, 0x8bc3, 0x8afa, 0x8a32, 0x896a, 0x88a1, 0x87d8, 0x8710, 0x8647, 0x857e, 0x84b5, 0x83ec, 0x8323, 0x825a, 0x8191, 0x80c8, 
    0x7fff, 0x7f36, 0x7e6d, 0x7da4, 0x7cdb, 0x7c12, 0x7b49, 0x7a80, 0x79b7, 0x78ee, 0x7826, 0x775d, 0x7694, 0x75cc, 0x7504, 0x743b, 
    0x7373, 0x72ab, 0x71e3, 0x711c, 0x7054, 0x6f8c, 0x6ec5, 0x6dfe, 0x6d37, 0x6c70, 0x6baa, 0x6ae3, 0x6a1d, 0x6957, 0x6891, 0x67cc, 
    0x6706, 0x6641, 0x657c, 0x64b8, 0x63f4, 0x6330, 0x626c, 0x61a8, 0x60e5, 0x6022, 0x5f60, 0x5e9d, 0x5ddb, 0x5d1a, 0x5c59, 0x5b98, 
    0x5ad7, 0x5a17, 0x5957, 0x5898, 0x57d8, 0x571a, 0x565b, 0x559d, 0x54e0, 0x5423, 0x5366, 0x52aa, 0x51ee, 0x5133, 0x5078, 0x4fbd, 
    0x4f03, 0x4e4a, 0x4d91, 0x4cd8, 0x4c20, 0x4b69, 0x4ab2, 0x49fb, 0x4945, 0x4890, 0x47db, 0x4726, 0x4672, 0x45bf, 0x450c, 0x445a, 
    0x43a9, 0x42f7, 0x4247, 0x4197, 0x40e8, 0x4039, 0x3f8b, 0x3ede, 0x3e31, 0x3d85, 0x3cda, 0x3c2f, 0x3b84, 0x3adb, 0x3a32, 0x398a, 
    0x38e2, 0x383c, 0x3795, 0x36f0, 0x364b, 0x35a7, 0x3504, 0x3461, 0x33bf, 0x331e, 0x327e, 0x31de, 0x313f, 0x30a1, 0x3004, 0x2f67, 
    0x2ecc, 0x2e30, 0x2d96, 0x2cfd, 0x2c64, 0x2bcc, 0x2b35, 0x2a9f, 0x2a0a, 0x2975, 0x28e1, 0x284f, 0x27bc, 0x272b, 0x269b, 0x260b, 
    0x257d, 0x24ef, 0x2462, 0x23d6, 0x234b, 0x22c1, 0x2238, 0x21af, 0x2128, 0x20a1, 0x201c, 0x1f97, 0x1f13, 0x1e90, 0x1e0e, 0x1d8d, 
    0x1d0d, 0x1c8e, 0x1c10, 0x1b93, 0x1b17, 0x1a9c, 0x1a21, 0x19a8, 0x1930, 0x18b9, 0x1842, 0x17cd, 0x1759, 0x16e6, 0x1673, 0x1602, 
    0x1592, 0x1523, 0x14b4, 0x1447, 0x13db, 0x1370, 0x1306, 0x129d, 0x1235, 0x11cf, 0x1169, 0x1104, 0x10a0, 0x103e, 0x0fdc, 0x0f7c, 
    0x0f1d, 0x0ebe, 0x0e61, 0x0e05, 0x0daa, 0x0d50, 0x0cf8, 0x0ca0, 0x0c4a, 0x0bf4, 0x0ba0, 0x0b4d, 0x0afb, 0x0aaa, 0x0a5a, 0x0a0b, 
    0x09be, 0x0971, 0x0926, 0x08dc, 0x0893, 0x084b, 0x0805, 0x07bf, 0x077b, 0x0738, 0x06f6, 0x06b5, 0x0675, 0x0637, 0x05fa, 0x05bd, 
    0x0582, 0x0549, 0x0510, 0x04d9, 0x04a2, 0x046d, 0x043a, 0x0407, 0x03d6, 0x03a5, 0x0376, 0x0348, 0x031c, 0x02f0, 0x02c6, 0x029d, 
    0x0275, 0x024e, 0x0229, 0x0205, 0x01e2, 0x01c0, 0x01a0, 0x0180, 0x0162, 0x0145, 0x012a, 0x010f, 0x00f6, 0x00de, 0x00c7, 0x00b2, 
    0x009d, 0x008a, 0x0078, 0x0068, 0x0058, 0x004a, 0x003d, 0x0031, 0x0027, 0x001e, 0x0016, 0x000f, 0x0009, 0x0005, 0x0002, 0x0000, 
    0x0000, 0x0000, 0x0002, 0x0005, 0x0009, 0x000f, 0x0016, 0x001e, 0x0027, 0x0031, 0x003d, 0x004a, 0x0058, 0x0068, 0x0078, 0x008a, 
    0x009d, 0x00b2, 0x00c7, 0x00de, 0x00f6, 0x010f, 0x012a, 0x0145, 0x0162, 0x0180, 0x01a0, 0x01c0, 0x01e2, 0x0205, 0x0229, 0x024e, 
    0x0275, 0x029d, 0x02c6, 0x02f0, 0x031c, 0x0348, 0x0376, 0x03a5, 0x03d6, 0x0407, 0x043a, 0x046d, 0x04a2, 0x04d9, 0x0510, 0x0549, 
    0x0582, 0x05bd, 0x05fa, 0x0637, 0x0675, 0x06b5, 0x06f6, 0x0738, 0x077b, 0x07bf, 0x0805, 0x084b, 0x0893, 0x08dc, 0x0926, 0x0971, 
    0x09be, 0x0a0b, 0x0a5a, 0x0aaa, 0x0afb, 0x0b4d, 0x0ba0, 0x0bf4, 0x0c4a, 0x0ca0, 0x0cf8, 0x0d50, 0x0daa, 0x0e05, 0x0e61, 0x0ebe, 
    0x0f1d, 0x0f7c, 0x0fdc, 0x103e, 0x10a0, 0x1104, 0x1169, 0x11cf, 0x1235, 0x129d, 0x1306, 0x1370, 0x13db, 0x1447, 0x14b4, 0x1523, 
    0x1592, 0x1602, 0x1673, 0x16e6, 0x1759, 0x17cd, 0x1842, 0x18b9, 0x1930, 0x19a8, 0x1a21, 0x1a9c, 0x1b17, 0x1b93, 0x1c10, 0x1c8e, 
    0x1d0d, 0x1d8d, 0x1e0e, 0x1e90, 0x1f13, 0x1f97, 0x201c, 0x20a1, 0x2128, 0x21af, 0x2238, 0x22c1, 0x234b, 0x23d6, 0x2462, 0x24ef, 
    0x257d, 0x260b, 0x269b, 0x272b, 0x27bc, 0x284f, 0x28e1, 0x2975, 0x2a0a, 0x2a9f, 0x2b35, 0x2bcc, 0x2c64, 0x2cfd, 0x2d96, 0x2e30, 
    0x2ecc, 0x2f67, 0x3004, 0x30a1, 0x313f, 0x31de, 0x327e, 0x331e, 0x33bf, 0x3461, 0x3504, 0x35a7, 0x364b, 0x36f0, 0x3795, 0x383c, 
    0x38e2, 0x398a, 0x3a32, 0x3adb, 0x3b84, 0x3c2f, 0x3cda, 0x3d85, 0x3e31, 0x3ede, 0x3f8b, 0x4039, 0x40e8, 0x4197, 0x4247, 0x42f7, 
    0x43a9, 0x445a, 0x450c, 0x45bf, 0x4672, 0x4726, 0x47db, 0x4890, 0x4945, 0x49fb, 0x4ab2, 0x4b69, 0x4c20, 0x4cd8, 0x4d91, 0x4e4a, 
    0x4f03, 0x4fbd, 0x5078, 0x5133, 0x51ee, 0x52aa, 0x5366, 0x5423, 0x54e0, 0x559d, 0x565b, 0x571a, 0x57d8, 0x5898, 0x5957, 0x5a17, 
    0x5ad7, 0x5b98, 0x5c59, 0x5d1a, 0x5ddb, 0x5e9d, 0x5f60, 0x6022, 0x60e5, 0x61a8, 0x626c, 0x6330, 0x63f4, 0x64b8, 0x657c, 0x6641, 
    0x6706, 0x67cc, 0x6891, 0x6957, 0x6a1d, 0x6ae3, 0x6baa, 0x6c70, 0x6d37, 0x6dfe, 0x6ec5, 0x6f8c, 0x7054, 0x711c, 0x71e3, 0x72ab, 
    0x7373, 0x743b, 0x7504, 0x75cc, 0x7694, 0x775d, 0x7826, 0x78ee, 0x79b7, 0x7a80, 0x7b49, 0x7c12, 0x7cdb, 0x7da4, 0x7e6d, 0x7f36
};

#define WAVEFORM_SIZE (sizeof(sin_table) / sizeof(uint16_t))
#define MAX_SIN 65535
#define FRACTION 256


#define CALCULATE_WAVEFORM(x) ((int)sin_table[x] * POWER / MAX_SIN)
#define ABS(x) ((x) > 0 ? (x) : (-(x)))

// 0 - 360 * FRACTION written to motors
int phase = 0;

// current change per frame
int phase_speed = 0;
// last wanted change per frame
int target_phase_speed = 0;
// last acceleration
int acceleration = 0;
int motor_timeout = 0;

volatile int tick = 0;


// radio parsing
void (*radio_function)();
#define RADIO_BUFSIZE (sizeof(PACKET_KEY) + PACKET_DATA)
volatile unsigned char receive_buf[RADIO_BUFSIZE];
volatile int radio_counter = 0;
volatile unsigned char radio_data = 0;
#define RADIO_TIMEOUT HZ
volatile int timeout_counter = 0;
// values from the radio
volatile uint8_t adc_raw = 0xff;
volatile uint8_t timelapse_code = 0xff;
volatile uint8_t control_valid = 0;
RCC_ClocksTypeDef RCC_ClocksStatus;

#define RADIO_CS_GPIO GPIOB
#define RADIO_CS_PIN GPIO_Pin_12
#define RADIO_SDO_GPIO GPIOB
#define RADIO_SDO_PIN GPIO_Pin_15
#define RADIO_CLK_GPIO GPIOB
#define RADIO_CLK_PIN GPIO_Pin_13

// write motor phase to hardware
void write_motor()
{
// reverse for gearbox
    int phase2 = -phase;
	int index1 = (phase2 * WAVEFORM_SIZE / 360 / FRACTION) % WAVEFORM_SIZE;
	int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;


	TIM5->CCR1 = CALCULATE_WAVEFORM(index1);
	TIM5->CCR2 = CALCULATE_WAVEFORM(index2);
	TIM5->CCR3 = CALCULATE_WAVEFORM(index3);
}



void get_key();
void get_packet()
{
    receive_buf[radio_counter++] = radio_data;
    if(radio_counter >= PACKET_DATA)
    {
        radio_counter = 0;
        radio_function = get_key;
        
        int i;
        int failed = 0;

// XOR the data key
        for(i = 0; i < PACKET_DATA; i++)
        {
            receive_buf[i] ^= DATA_KEY[i];
        }

        for(i = 2; i < PACKET_DATA; i += 2)
        {
// reject an invalid packet
            if(receive_buf[i] != receive_buf[0] ||
                receive_buf[i + 1] != receive_buf[1])
            {
                failed = 1;
                break;
            }
        }

        if(!failed)
        {
//TRACE2
//print_text("tick=");
//print_number(tick);
// print_number(receive_buf[0]);
// print_text("blink_counter=");
// print_number(receive_buf[1]);
// print_lf();

            TOGGLE_PIN(LED_GPIO, LED_PIN);
            timeout_counter = 0;
//print_text("got chan=");
//print_number(current_channel);
//print_lf();
            timelapse_code = receive_buf[0];
            adc_raw = receive_buf[1];
            control_valid = 1;
//TRACE2
//print_text("timelapse_code=");
//print_number(timelapse_code);
//print_text("adc_raw=");
//print_number(adc_raw);

// schedule the next hop to the start time of this packet + HOP duration + HOP_LAG
            scanning = 0;
            missed_packets = 0;
            next_hop = packet_tick + HZ / HOP_HZ - HOP_LAG;
        }
    }
}

void get_key()
{
    if(radio_data == PACKET_KEY[radio_counter])
    {
        if(radio_counter == 0)
        {
            packet_tick = tick;
        }
    
        radio_counter++;
        if(radio_counter >= sizeof(PACKET_KEY))
        {
            radio_function = get_packet;
            radio_counter = 0;
        }
    }
    else
    if(radio_data == PACKET_KEY[0])
    {
        packet_tick = tick;
        radio_counter = 1;
    }
    else
    {
        radio_counter = 0;
    }
}

// write radio SPI.  TODO: use hardware
void write_radio(uint16_t data)
{
// print_text("write_radio ");
// print_hex(data);
// print_lf();
    CLEAR_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);

    int i;
    for(i = 0; i < 16; i++)
    {
        if(data & 0x8000)
        {
            SET_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        else
        {
            CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        data <<= 1;
// need the delay if the system clock is over 16Mhz
//        udelay(1);
        SET_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
//        udelay(1);
        CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    }
    
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
}

void init_radio()
{
    TRACE
    GPIO_InitTypeDef GPIO_InitStructure;
	radio_function = get_key;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#define RADIO_RX_PIN 11
#define RADIO_TX_PIN 10
	GPIO_PinAFConfig(GPIOB, RADIO_RX_PIN, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, RADIO_TX_PIN, GPIO_AF_USART3);

// RX enabled
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = 1 << RADIO_RX_PIN;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

// TX enabled
//  GPIO_InitStructure.GPIO_Pin = 1 << RADIO_TX_PIN;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 100000;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx /* | USART_Mode_Tx */;
/* USART configuration */
  	USART_Init(USART3, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART3, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

// initialize radio SPI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RADIO_CS_PIN |
        RADIO_SDO_PIN |
        RADIO_CLK_PIN;
    GPIO_Init(RADIO_CS_GPIO, &GPIO_InitStructure);
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
    CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
    CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    
    



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
// turn on the transmitter to tune.  Not necessary to receive.
//    write_radio(PMCREG | 0x0020);

// warm up
//     int tick1 = tick;
//     while(tick - tick1 < HZ / 100)
//     {
//         ;
//     }

// receive mode
    write_radio(PMCREG | 0x0080);

}



void next_channel()
{
    current_channel++;
    if(current_channel >= TOTAL_CHANNELS)
    {
        current_channel = 0;
    }
//print_text("tune chan=");
//print_number(current_channel);
//print_lf();
    write_radio(CFSREG(channels[current_channel]));
}







void init_motor()
{
    TRACE
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | 
		GPIO_Pin_1 | 
		GPIO_Pin_2;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);



	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = RCC_ClocksStatus.SYSCLK_Frequency / 
        (PWM_PERIOD + 1) /
        32000 - 1;

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
  	TIM_Cmd(TIM5, ENABLE);
 	TIM_CtrlPWMOutputs(TIM5, ENABLE);

    write_motor();

// Enable pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = ENABLE_PIN;
	GPIO_Init(ENABLE_GPIO, &GPIO_InitStructure);
	CLEAR_PIN(ENABLE_GPIO, ENABLE_PIN);

}



void init_watchdog()
{
  /* Guess the LSI frequency by running it without petting the watchdog */
  int LsiFreq = 32768;




  /* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
     dispersion) */
  /* Enable write access to IWDG_PR and IWDG_RLR registers */
  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

  /* IWDG counter clock: LSI/32 */
  IWDG_SetPrescaler(IWDG_Prescaler_32);
// 1 second is now supposed to be LsiFreq / 32
  IWDG_SetReload(LsiFreq * 3 / 32);

  /* Reload IWDG counter */
  IWDG_ReloadCounter();

  /* Enable IWDG (the LSI oscillator will be enabled by hardware) */
  IWDG_Enable();
}



// DEBUG uart
void USART6_IRQHandler(void)
{
	unsigned char c = USART6->DR;
	uart.input = c;
	uart.got_input = 1;
}

// radio uart
void USART3_IRQHandler(void)
{
	radio_data = USART3->DR;
	radio_function();
}



// TIM10 wraps at HZ
void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
		
//        TOGGLE_PIN(LED_GPIO, LED_PIN);

		tick++;


// reset the control code
        if(timeout_counter >= RADIO_TIMEOUT)
        {
            control_valid = 0;
        }
        else
        {
            timeout_counter++;
        }


        if(tick >= next_hop)
        {
            need_hop = 1;
        }

	}
}

int main(void)
{
	int i, j;
 
 
 
// switch system clock to non PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
// disable mane PLL
//    RCC_PLLCmd(DISABLE);

// clockspeed = 8Mhz / PLLM * PLLN / PLL_P
// 168Mhz = 63mA
// 32Mhz = 45mA
// reduce clockspeed to reduce power consumption
// mane PLL
#define PLL_M 8
#define PLL_N 32
#define PLL_P 2
#define PLL_Q 7
//     RCC->PLLCFGR = PLL_M | 
// 		(PLL_N << 6) | 
// 		(((PLL_P >> 1) -1) << 16) |
//         (RCC_PLLCFGR_PLLSRC_HSI) | 
// 		(PLL_Q << 24);
//     RCC_PLLCmd(ENABLE);



    SystemCoreClockUpdate();

	init_linux();

/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB, 
		ENABLE);

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();


	RCC_GetClocksFreq(&RCC_ClocksStatus);
	print_text("Welcome to cam panner\n");
	flush_uart();
	print_text("SYSCLK_Frequency=");
	print_number(RCC_ClocksStatus.SYSCLK_Frequency);
	print_lf();
	print_text("SystemCoreClock=");
	print_number(SystemCoreClock);
	print_lf();
	print_text("HCLK_Frequency=");
	print_number(RCC_ClocksStatus.HCLK_Frequency);
	print_lf();
	print_text("PCLK1_Frequency=");
	print_number(RCC_ClocksStatus.PCLK1_Frequency);
	print_lf();
	print_text("PCLK2_Frequency=");
	print_number(RCC_ClocksStatus.PCLK2_Frequency);
	print_lf();

	flush_uart();
    init_watchdog();
	flush_uart();

// general purpose timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = RCC_ClocksStatus.SYSCLK_Frequency / 100 / HZ - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 100;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);






 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);


 	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
 	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM10, TIM_IT_Update, ENABLE);


	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

// debug pin
// 	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
// 	GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);
// 	CLEAR_PIN(DEBUG_GPIO, DEBUG_PIN);

// LED pin
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	SET_PIN(LED_GPIO, LED_PIN);

	init_motor();
    init_radio();

    TRACE
// wait a while
    while(tick < HZ)
    {
        ;
    }
    TRACE

//    print_text("Enabling motor\n");
//   	SET_PIN(ENABLE_GPIO, ENABLE_PIN);


    int seconds = 0;
    int next_frame = 0;
    int prev_frame = 0;
    int debug_count = 0;
	while(1)
	{
		handle_uart();



        if(tick >= next_frame || tick < prev_frame)
        {
            prev_frame = tick;
            next_frame = prev_frame + HZ / FRAME_HZ;
            debug_count++;
            debug_count = debug_count % 10;

//             print_text("next_frame=");
//             print_number(next_frame);
//             print_lf();


// update speed based on analog value
            int adc = adc_raw;
            if(control_valid && adc < MIN_RIGHT)
            {
                acceleration = ACCELERATION;
//                 acceleration = SLOW_ACCELERATION +
//                     (FAST_ACCELERATION - SLOW_ACCELERATION) *
//                     (adc - MIN_RIGHT) /
//                     (MAX_RIGHT - MIN_RIGHT);
                target_phase_speed = SLOW_SPEED + 
                    (FAST_SPEED - SLOW_SPEED) * 
                    (adc - MIN_RIGHT) /
                    (MAX_RIGHT - MIN_RIGHT);

                if(target_phase_speed > FAST_SPEED)
                {
                    target_phase_speed = FAST_SPEED;
                }
                if(phase_speed > target_phase_speed)
                {
// wind it back based on acceleration to avoid instantly jumping back to slow speed
                    phase_speed -= acceleration / FRAME_HZ;
                    if(phase_speed < target_phase_speed)
                    {
                        phase_speed = target_phase_speed;
                    }
                }
                else
                {
                    phase_speed += acceleration / FRAME_HZ;
                    if(phase_speed > target_phase_speed)
                    {
                        phase_speed = target_phase_speed;
                    }
                }
            }
            else
            if(control_valid && adc > MIN_LEFT)
            {
                acceleration = ACCELERATION;
//                 acceleration = SLOW_ACCELERATION +
//                     (FAST_ACCELERATION - SLOW_ACCELERATION) *
//                     (MIN_LEFT - adc) /
//                     (MIN_LEFT - MAX_LEFT);
                target_phase_speed = SLOW_SPEED + 
                    (FAST_SPEED - SLOW_SPEED) * 
                    (MIN_LEFT - adc) /
                    (MIN_LEFT - MAX_LEFT);
                if(target_phase_speed > FAST_SPEED)
                {
                    target_phase_speed = FAST_SPEED;
                }
                target_phase_speed = -target_phase_speed;
                if(phase_speed < target_phase_speed)
                {
// wind it back based on acceleration to avoid instantly jumping back to slow speed
                    phase_speed += acceleration / FRAME_HZ;
                    if(phase_speed > target_phase_speed)
                    {
                        phase_speed = target_phase_speed;
                    }
                }
                else
                {
                    phase_speed -= acceleration / FRAME_HZ;
                    if(phase_speed < target_phase_speed)
                    {
                        phase_speed = target_phase_speed;
                    }
                }
            }
            else
            {
                target_phase_speed = 0;
// wind phase speed back to 0 based on acceleration
                if(phase_speed > 0)
                {
                    phase_speed -= acceleration / FRAME_HZ;
                    if(phase_speed < 0)
                    {
                        phase_speed = 0;
                    }
                }
                else
                if(phase_speed < 0)
                {
                    phase_speed += acceleration / FRAME_HZ;
                    if(phase_speed > 0)
                    {
                        phase_speed = 0;
                    }
                }
                
            }

//             if(debug_count == 0)
//             {
//                  print_text("adc=");
//                  print_number(adc_raw);
//                  print_text("timelapse_code=");
//                  print_hex(timelapse_code);
//                  print_text("phase_speed=");
//                  print_fixed(target_phase_speed);
//                  print_text("acceleration=");
//                  print_fixed(acceleration);
//                  print_lf();
//             }

// add timelapse speed to phase speed.  Reverse direction for gearbox.
            const int timelapse_speeds[] = 
            {
// TIMELAPSE_FAST_LEFT
                -FAST_TIMELAPSE,
// TIMELAPSE_FAST_RIGHT
                FAST_TIMELAPSE,
                0,
                0,
                0,
                0
            };
            int total_timelapse_speeds = sizeof(timelapse_speeds) / sizeof(int);
            int timelapse_speed = 0;
            if(control_valid && 
                timelapse_code >= 0 && 
                timelapse_code < total_timelapse_speeds)
            {
                timelapse_speed = timelapse_speeds[timelapse_code];
            }

            int new_phase = phase + (phase_speed + timelapse_speed) / FRAME_HZ;
// wrap angle
            while(new_phase < 0)
            {
                new_phase += 360 * FRACTION;
            }
            new_phase %= 360 * FRACTION;
            
            if(new_phase != phase)
            {
// reset motor timeout
                motor_timeout = 0;
                phase = new_phase;
// enable motor
                if(PIN_IS_CLEAR(ENABLE_GPIO, ENABLE_PIN))
                {
                    SET_PIN(ENABLE_GPIO, ENABLE_PIN);
//TRACE2
                }
//print_fixed(phase);
//print_lf();
                write_motor();
            }
            else
            {
                if(motor_timeout >= MOTOR_TIMEOUT)
                {
// disable motor
                    if(PIN_IS_SET(ENABLE_GPIO, ENABLE_PIN))
                    {
                        CLEAR_PIN(ENABLE_GPIO, ENABLE_PIN);
//TRACE2
                    }
                }
                else
                {
                    motor_timeout += HZ / FRAME_HZ;
                }
            }
        }
        

// check clock
        if(tick / HZ != seconds)
        {
            seconds = tick / HZ;
//             print_text("seconds=");
//             print_number(seconds);
//             print_lf();
        }


// frequency hopping
        if(need_hop)
        {
            need_hop = 0;
            if(scanning)
            {
                next_hop = tick + HZ / SCAN_HZ;
            }
            else
            {
// always incremented, whether we got a packet or not
                missed_packets++;
// too many hops without a packet.  Go to scanning mode
                if(missed_packets > MAX_MISSED_PACKETS)
                {
                    scanning = 1;
                    next_hop = tick + HZ / SCAN_HZ;
                }
                else
                {
                    next_hop = tick + HZ / HOP_HZ;
                }
            }

//             if(missed_packets > 1)
//             {
//                 print_text("missed chan=");
//                 print_number(current_channel);
//                 print_lf();
//             }
//TRACE2
//print_text("scanning=");
//print_number(scanning);
//print_text("current_channel=");
//print_number(current_channel);
            next_channel();
        }

        
        PET_WATCHDOG
	}
	
		
}





