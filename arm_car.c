/*
 * STM32F4 Wifi flight controller
 * Copyright (C) 2012-2014 Adam Williams <broadcast at earthling dot net>
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

// make arm

// uart_programmer car.bin




#include "arm_car.h"
#include "arm_math.h"
#include "cc1101.h"
#include "imu.h"
#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include <math.h>


#define SYNC_CODE 0xe5


#define DEBUG_PIN GPIO_Pin_4
#define DEBUG_GPIO GPIOB
#define WAVEFORM_SIZE (sizeof(sin_table) / sizeof(uint16_t))
#define MAX_PERIOD 65535
#define STEERING_PERIOD 4096
// period to switch to COMMUTATING mode
// purely to keep the tires from slipping
#define MIN_STEPPER_PERIOD 1024
// Amount to change phase
#define FAST_STEERING 8
#define SLOW_STEERING 2
// slow turn rate per second in degrees
#define SLOW_TURN_RATE 40.0f
// packets per second
#define PACKET_RATE 40
#define MOTOR_STEP 10
#define PWM_PERIOD 1024
#define MAX_PWM PWM_PERIOD
#define PHASE_PRESCALE 128
// motors
#define LEFT_MOTOR 1
#define RIGHT_MOTOR 0
// motor modes
#define OFF 0
#define STEPPER 1
#define COMMUTATING 2
#define I_DOWNSAMPLE (NAV_HZ / 100)
#define HALL_DEADBAND 8

#define THROTTLE_MIN 0x100
#define THROTTLE_MAX 0xff00
// analog range
#define STEERING_MID 0x8000
// deadband
#define STEERING_MIN 0x800
#define STEERING_MAX 0x7f00
#define STEERING_FRACTION 75

#define MAX_SIN 65535
#define FIX_PHASE(x) \
	while(x < 0) x += 360; \
	while(x >= 360) x -= 360;

#define BATTERY_OVERSAMPLE 2048
// 9.5 - 7m46s/mile
#define POWER_VOLTAGE 9.5f
// 9.4 - 
#define POWER_VOLTAGE 9.4f
// 9.2 - 8m4s/mile
//#define POWER_VOLTAGE 9.2f
// 9.0 - 8m16s/mile
//#define POWER_VOLTAGE 9.0f
#define STEPPER_VOLTAGE 6.0f





car_t car;

// 3 sensor values
// 3 H bridge values
#define COMMUTATION_SIZE 6
const int8_t rev_commutation_table[] = 
{
	1, 0, 1,	 1,  0, -1,
	1, 0, 0,	 1, -1,  0,
	1, 1, 0,	 0, -1,  1,
	0, 1, 0,	-1,  0,  1,
	0, 1, 1,	-1,  1,  0,
	0, 0, 1,	 0,  1, -1,
};

const int8_t fwd_commutation_table[] = 
{
	0, 1, 1,	 1, -1,  0,
	0, 0, 1,	 0, -1,  1,
	1, 0, 1,	-1,  0,  1,
	1, 0, 0,	-1,  1,  0,
	1, 1, 0,	 0,  1, -1,
	0, 1, 0,	 1,  0, -1,
};

#define TOTAL_SENSOR_ORDERS 6
const int8_t left_sensor_table[TOTAL_SENSOR_ORDERS][SENSORS] = 
{
	{ ADC_Channel_13, ADC_Channel_1,  ADC_Channel_0 },
	{ ADC_Channel_13, ADC_Channel_0,  ADC_Channel_1 },
	{ ADC_Channel_1,  ADC_Channel_13, ADC_Channel_0 },
	{ ADC_Channel_0,  ADC_Channel_13, ADC_Channel_1 },
	{ ADC_Channel_1,  ADC_Channel_0,  ADC_Channel_13 },
	{ ADC_Channel_0,  ADC_Channel_1,  ADC_Channel_13 }
};

const int8_t right_sensor_table[TOTAL_SENSOR_ORDERS][SENSORS] = 
{
	{ ADC_Channel_11, ADC_Channel_12,  ADC_Channel_10 },
	{ ADC_Channel_11, ADC_Channel_10,  ADC_Channel_12 },
	{ ADC_Channel_12, ADC_Channel_11,  ADC_Channel_10 },
	{ ADC_Channel_10, ADC_Channel_11,  ADC_Channel_12 },
	{ ADC_Channel_12, ADC_Channel_10,  ADC_Channel_11 },
	{ ADC_Channel_10, ADC_Channel_12,  ADC_Channel_11 }
};


const uint16_t sin_table[] = 
{
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

int calculate_waveform(int x, int motor)
{
 	int result = (int)sin_table[x] * car.motor[motor].power / MAX_SIN;
	CLAMP(result, 0, MAX_PWM);
	return result;
}


void enable_phase(int motor_number, int phase)
{
	motor_t *motor = &car.motor[motor_number];
	
	if(car.mode == OFF)
	{
// break mode
		CLEAR_PIN(motor->in_gpio[phase], motor->in_pin[phase]);
		SET_PIN(motor->en_gpio[phase], motor->en_pin[phase]);
	}
	else
	if(car.mode == COMMUTATING)
	{
		if(!motor->disabled[phase] && motor->pwm[phase] > 0) 
		{
			if(motor->voltage[phase])
			{
				SET_PIN(motor->in_gpio[phase], motor->in_pin[phase]);
			}
			else
			{
				CLEAR_PIN(motor->in_gpio[phase], motor->in_pin[phase]);
			}
			
			SET_PIN(motor->en_gpio[phase], motor->en_pin[phase]);
		}
		else
		{
			CLEAR_PIN(motor->en_gpio[phase], motor->en_pin[phase]);
		}
		
		
		
	}
	else
	if(car.mode == STEPPER)
	{
		if(motor->pwm[phase] > 0) 
		{
			SET_PIN(motor->in_gpio[phase], motor->in_pin[phase]);
		}
		SET_PIN(motor->en_gpio[phase], motor->en_pin[phase]);
	}
}

void disable_phase(int motor_number, int phase)
{
	motor_t *motor = &car.motor[motor_number];
	
// The interrupt was missed until after the next rising edge
	if(motor->pwm_timer->CNT < motor->pwm[phase]) return;
	
	if(car.mode == COMMUTATING)
	{
		if(motor->pwm[phase] < MAX_PWM) 
		{
			CLEAR_PIN(motor->en_gpio[phase], motor->en_pin[phase]);
		}
	}
	else
	if(car.mode == STEPPER)
	{
		if(motor->pwm[phase] < MAX_PWM) 
		{
			CLEAR_PIN(motor->in_gpio[phase], motor->in_pin[phase]);
		}
	}
}

// calculate motor PWM
void write_motor(int number)
{
	int i;
	motor_t *motor = &car.motor[number];
	
	FIX_PHASE(motor->phase);

	if(car.mode == OFF)
	{
// break mode
		motor->pwm[0] = 0;
		motor->pwm[1] = 0;
		motor->pwm[2] = 0;
		motor->disabled[0] = 0;
		motor->disabled[1] = 0;
		motor->disabled[2] = 0;
	}
	else
	if(car.mode == COMMUTATING)
	{
		motor->pwm[0] = motor->power;
		motor->pwm[1] = motor->power;
		motor->pwm[2] = motor->power;
		
		for(i = 0; i < PHASES; i++)
		{
			if(motor->pwm_timer->CNT < motor->pwm[i])
			{
				enable_phase(number, i);
			}
			else
			{
				disable_phase(number, i);
			}
		}
	}
	else
	{
		int index1 = (int)((motor->phase * WAVEFORM_SIZE / 360)) % WAVEFORM_SIZE;
		int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
		int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;


/*
* TRACE2
* print_number(calculate_waveform(index1));
* print_number(calculate_waveform(index2));
* print_number(calculate_waveform(index3));
*/


		motor->pwm[0] = calculate_waveform(index1, number);
		motor->pwm[1] = calculate_waveform(index2, number);
		motor->pwm[2] = calculate_waveform(index3, number);
	}
//car.pwm[0][0] = 0;
//car.pwm[0][1] = 0;
//car.pwm[0][2] = 0;
	
}


void write_motors()
{
	int i = 0;
	
	for(i = 0; i < MOTORS; i++)
	{
		write_motor(i);
	}
	
}

float init_pid(pid_t *pid, 
	float p_gain, 
	float i_gain,
	float d_gain,
	float i_limit,
	float o_limit)
{
	pid->p_gain = p_gain;
	pid->i_gain = i_gain;
	pid->d_gain = d_gain;
	pid->i_limit = i_limit;
	pid->o_limit = o_limit;
}

float do_pid(pid_t *pid, float p_error, float d_error)
{
	float p_result = p_error * pid->p_gain;
	float d_result = d_error * pid->d_gain;

	pid->error_accum += p_error;
	pid->counter++;
	if(pid->counter >= I_DOWNSAMPLE)
	{
// average of all errors
		pid->error_accum /= pid->counter;
// I factor
		pid->accum += pid->error_accum * pid->i_gain;
		CLAMP(pid->accum, -pid->i_limit, pid->i_limit);
		pid->counter = 0;
		pid->error_accum = 0;
	}

	float result = p_result + d_result + pid->accum;
	CLAMP(result, -pid->o_limit, pid->o_limit);
	return result;
}

void reset_pid(pid_t *pid)
{
	pid->error_accum = 0;
	pid->accum = 0;
	pid->counter = 0;
}

void handle_commutation(int motor_number)
{
	int i, j;
	motor_t *motor = &car.motor[motor_number];
	if(car.mode == COMMUTATING)
	{
		const int8_t *commutation_table;
		
		int reverse = 0;
		if(car.throttle_reverse) reverse = !reverse;
		if(motor->commutation_direction < 0) reverse = !reverse;
		
		
		if(!reverse)
			commutation_table = fwd_commutation_table;
		else
			commutation_table = rev_commutation_table;



		int got_it = 0;		
		for(i = 0; i < COMMUTATION_SIZE; i++)
		{
			got_it = 1;
			for(j = 0; j < SENSORS; j++)
			{
				if(commutation_table[i * 6 + j] != motor->sensors[j])
				{
					got_it = 0;
				}
			}

			if(got_it) break;
		}

// glitched out of table
		if(!got_it)
		{
//			TRACE2
//			print_text("glitch motor=");
//			print_number(motor_number);
//			for(i = 0; i < SENSORS; i++)
//				print_number(motor->sensors[i]);
		}
		else
// DEBUG
//		if(motor->commutations < 10)
		{
			for(j = 0; j < PHASES; j++)
			{
				if(commutation_table[i * 6 + SENSORS + j] == 0)
				{
					motor->disabled[j] = 1;
				}
				else
				if(commutation_table[i * 6 + SENSORS + j] == -1)
				{
					motor->disabled[j] = 0;
					motor->voltage[j] = 0;
				}
				else
				if(commutation_table[i * 6 + SENSORS + j] == 1)
				{
					motor->disabled[j] = 0;
					motor->voltage[j] = 1;
				}
			}

// have to update PWM pins here to get the illusion of a single continuous PWM field
			if(motor->commutation_slot != i)
			{
				write_motor(motor_number);
			}

			motor->commutation_slot = i;
			motor->commutations++;
		}

		write_motor(motor_number);
	}
}

// handle stepper states.  Both motors are synchronized
void handle_motors()
{
	int i;
	if(!imu.need_gyro_center)
	{
		motor_t *left_motor = &car.motor[LEFT_MOTOR];
		motor_t *right_motor = &car.motor[RIGHT_MOTOR];

		if(car.throttle <= THROTTLE_MIN)
		{
// reset
			car.mode = OFF;
			car.period = MAX_PERIOD;
			left_motor->power = 0;
			right_motor->power = 0;
			
			int power = 0;
			int period = 0;
			float phase = 0;
			int enabled = 0;

// test if steering
			if(car.steering < STEERING_MID - STEERING_MAX)
			{
				phase = -FAST_STEERING;
				enabled = 1;
			}
			else
			if(car.steering < STEERING_MID - STEERING_MIN)
			{
				phase = -SLOW_STEERING;
				enabled = 1;
			}
			else
			if(car.steering > STEERING_MID + STEERING_MAX)
			{
				phase = FAST_STEERING;
				enabled = 1;
			}
			else
			if(car.steering > STEERING_MID + STEERING_MIN)
			{
				phase = SLOW_STEERING;
				enabled = 1;
			}
			
			if(enabled)
			{
				car.mode = STEPPER;
				for(i = 0; i < MOTORS; i++)
				{
					car.motor[i].power = car.stepper_power;
					car.period = STEERING_PERIOD;
					car.motor[i].phase += phase * 
						car.motor[i].stepper_direction * 
						car.motor[i].turn_sign;
				}
			}

			TIM1->CCR1 = car.period;
			
			write_motors();
		}
/*
 * 		else
 * // car.throttle > THROTTLE_MIN
 * 		{
 * // acceleration phase
 * 			if(car.mode == STEPPER)
 * 			{
 * 				int sign = car.throttle_reverse ? -1 : 1;
 * 				for(i = 0; i < MOTORS; i++)
 * 				{
 * 					car.motor[i].phase += MOTOR_STEP * 
 * 						sign * 
 * 						car.motor[i].stepper_direction;
 * 					car.motor[i].power = car.stepper_power;
 * 				}
 * 				
 * 				TIM1->CCR1 = car.period;
 * 
 * // acceleration done
 * 				if(car.period <= MIN_STEPPER_PERIOD)
 * 				{
 * 					car.mode = COMMUTATING;
 * 					left_motor->do_ramp = 1;
 * 					right_motor->do_ramp = 1;
 * 					left_motor->power = car.operating_power0;
 * 					right_motor->power = car.operating_power0;
 * 					left_motor->commutations = 0;
 * 					right_motor->commutations = 0;
 * 					handle_commutation(LEFT_MOTOR);
 * 					handle_commutation(RIGHT_MOTOR);
 * 				}
 * 		 		write_motors();
 * 			}
 * 		}
 */
	}
}

// for stepper mode
void update_period()
{
	int i;
	if(car.throttle > THROTTLE_MIN)
	{
		if(car.mode == STEPPER)
		{

// have to change the period here, so the update rate is constant
			if(car.period > MIN_STEPPER_PERIOD)
			{
				car.period -= car.period / 8;
			}

		}
		else
		if(car.mode == COMMUTATING)
		{
			for(i = 0; i < MOTORS; i++)
			{
				motor_t *motor = &car.motor[i];
				if(motor->do_ramp && motor->power < car.operating_power)
				{
					motor->power += 8;
				}

				if(motor->power > car.operating_power)
				{
					motor->power = car.operating_power;
					motor->do_ramp = 0;
				}
/*
 * if(i == 0)
 * {
 * TRACE2
 * print_number(motor->power);
 * }
 */
			}
		}
		
	}
	
}


void handle_battery()
{
	DISABLE_INTERRUPTS
	int battery = car.battery;
	car.got_battery = 0;
	ENABLE_INTERRUPTS
	float voltage = battery * 8.58f / 984.0f;


	if(voltage > POWER_VOLTAGE) 
	{
		car.operating_power = MAX_PWM * POWER_VOLTAGE / voltage;
	}
	else
	{
		car.operating_power = MAX_PWM;
	}
	
	if(voltage > STEPPER_VOLTAGE)
	{
		car.stepper_power = MAX_PWM * STEPPER_VOLTAGE / voltage;
	}
	else
	{
		car.stepper_power = MAX_PWM;
	}

TRACE2
print_number(battery);
print_float(voltage);
print_number(car.operating_power);
print_number(car.stepper_power);

}

void handle_battery_adc()
{
	ADC3->SR = ~ADC_FLAG_EOC;
	
	car.battery_accum += ADC3->DR;
	car.battery_count++;
	if(car.battery_count >= BATTERY_OVERSAMPLE)
	{
		car.battery = car.battery_accum / car.battery_count;
		car.battery_accum = 0;
		car.battery_count = 0;
		car.got_battery = 1;
	}
	
	ADC_SoftwareStartConv(ADC3);
}

void handle_motor_adc(int motor_number)
{
	motor_t *motor = &car.motor[motor_number];
	int i;
	motor->adc->SR = ~ADC_FLAG_EOC;
	
	motor->adc_results[motor->adc_channel] = motor->adc->DR;

	if(motor->adc_throwaway > 100)
	{
		if(motor->adc->DR > motor->adc_max[motor->adc_channel])
			motor->adc_max[motor->adc_channel] = motor->adc->DR;
		if(motor->adc->DR < motor->adc_min[motor->adc_channel])
			motor->adc_min[motor->adc_channel] = motor->adc->DR;

		for(i = 0; i < SENSORS; i++)
		{
			if(motor->sensors[i])
			{
				if(motor->adc_results[i] < motor->threshold - HALL_DEADBAND)
					motor->sensors[i] = 0;
			}
			else
			{
				if(motor->adc_results[i] >= motor->threshold + HALL_DEADBAND)
					motor->sensors[i] = 1;
			}
		}
	}
	else
	{
		motor->adc_throwaway++;
	}
	
	motor->adc_channel++;
	if(motor->adc_channel >= SENSORS)
		motor->adc_channel = 0;

	ADC_RegularChannelConfig(motor->adc, 
		motor->adc_channels[motor->adc_channel], 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_SoftwareStartConv(motor->adc);

	handle_commutation(motor_number);
}

// phase timer for stepping wrapped
void TIM1_CC_IRQHandler()
{
	if(TIM1->SR & TIM_FLAG_CC1)
	{
		TIM1->SR = ~TIM_FLAG_CC1;
		TIM1->CNT = 0;
//TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);
		handle_motors();
	}
}

void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;

		car.timer_high++;

// Update shutdown timer
		if(car.shutdown_timeout > 0)
		{
			car.shutdown_timeout--;
		}

		update_period();

//		car.debug_counter++;
//		if(car.debug_counter >= 10)
//		{
//			int i;
//			motor_t *left_motor = &car.motor[LEFT_MOTOR];
//			motor_t *right_motor = &car.motor[RIGHT_MOTOR];
//
//			car.debug_counter = 0;
//
//				TRACE2
//				print_number(right_motor->voltage[0]);
//				print_number(right_motor->voltage[1]);
//				print_number(right_motor->voltage[2]);
//				for(i = 0; i < SENSORS; i++)
//					print_number(left_motor->sensors[i]);
//		}

//			TRACE2
//			print_number(right_motor->adc_min[0]);
//			print_number(right_motor->adc_max[0]);
//			print_number(right_motor->adc_min[1]);
//			print_number(right_motor->adc_max[1]);
//			print_number(right_motor->adc_min[2]);
//			print_number(right_motor->adc_max[2]);
//			print_number((right_motor->adc_min[0] + right_motor->adc_max[0]) / 2);
//			print_number((right_motor->adc_min[1] + right_motor->adc_max[1]) / 2);
//			print_number((right_motor->adc_min[2] + right_motor->adc_max[2]) / 2);
//			print_number(car.commutations[0]);
//			print_number(car.crossing_time[0]);
//			print_number(car.period);
//			print_text("rpm=");
//			print_number(4000.0f * 930.0f / car.period_lowpass);


		
	}
}



void ADC_IRQHandler()
{
//	TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);

	if((car.motor[LEFT_MOTOR].adc->SR & ADC_FLAG_EOC) != 0)
	{
		handle_motor_adc(LEFT_MOTOR);
	}

	if((car.motor[RIGHT_MOTOR].adc->SR & ADC_FLAG_EOC) != 0)
	{
		handle_motor_adc(RIGHT_MOTOR);
	}

// battery
	if((ADC3->SR & ADC_FLAG_EOC) != 0)
	{
		handle_battery_adc();
	}
}


void handle_controls()
{
	motor_t *left_motor = &car.motor[LEFT_MOTOR];
	motor_t *right_motor = &car.motor[RIGHT_MOTOR];
	int manual_steering = 0;

	imu.got_ahrs = 0;
	
	DISABLE_INTERRUPTS

	if(car.throttle > THROTTLE_MIN &&
		car.mode == COMMUTATING)
	{
// steering while driving
		if(car.steering < STEERING_MID - STEERING_MAX)
		{
			if(car.throttle_reverse)
				right_motor->power = car.operating_power * STEERING_FRACTION / 100;
			else
				left_motor->power = car.operating_power * STEERING_FRACTION / 100;
			imu.current_heading = 0;
			manual_steering = 1;
		}
		else
		if(car.steering > STEERING_MID + STEERING_MAX)
		{
			if(car.throttle_reverse)
				left_motor->power = car.operating_power * STEERING_FRACTION / 100;
			else
				right_motor->power = car.operating_power * STEERING_FRACTION / 100;
			imu.current_heading = 0;
			manual_steering = 1;
		}
		else
		if(car.steering < STEERING_MID - STEERING_MIN)
		{
			imu.current_heading += TO_RAD(SLOW_TURN_RATE / NAV_HZ);
		}
		else
		if(car.steering > STEERING_MID + STEERING_MIN)
		{
			imu.current_heading -= TO_RAD(SLOW_TURN_RATE / NAV_HZ);
		}
		else
		{
			left_motor->power = car.operating_power;
			right_motor->power = car.operating_power;
		}

		if(!manual_steering)
		{

			int result = do_pid(&car.heading_pid, 
				-imu.current_heading, 
				-imu.gyro_z_centered / imu.angle_to_gyro / NAV_HZ);

			if(car.throttle_reverse) result *= -1;

			if(result < 0)
			{
				left_motor->power = car.operating_power +
					result;
			}
			else
			{
				right_motor->power = car.operating_power -
					result;
			}

			CLAMP(left_motor->power, 0, MAX_PWM);
			CLAMP(right_motor->power, 0, MAX_PWM);

//			car.debug_counter++;
//			if(car.debug_counter >= 100)
//			{
//				car.debug_counter = 0;
//
//				TRACE2
//				print_number(left_motor->power);
//				print_number(right_motor->power);
//			}
		}
		else
		{
			reset_pid(&car.heading_pid);
		}
	}

// DEBUG
//left_motor->power = 0;

	ENABLE_INTERRUPTS
}



void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_FLAG_Update)
	{
		TIM2->SR = ~TIM_FLAG_Update;
		TIM2->CCR1 = car.motor[LEFT_MOTOR].pwm[0];
		TIM2->CCR2 = car.motor[LEFT_MOTOR].pwm[1];
		TIM2->CCR3 = car.motor[LEFT_MOTOR].pwm[2];

		enable_phase(LEFT_MOTOR, 0);
		enable_phase(LEFT_MOTOR, 1);
		enable_phase(LEFT_MOTOR, 2);
	}
	
	if(TIM2->SR & TIM_FLAG_CC1)
	{
		TIM2->SR = ~TIM_FLAG_CC1;
		disable_phase(LEFT_MOTOR, 0);
	}

	if(TIM2->SR & TIM_FLAG_CC2)
	{
		TIM2->SR = ~TIM_FLAG_CC2;
		disable_phase(LEFT_MOTOR, 1);
	}

	if(TIM2->SR & TIM_FLAG_CC3)
	{
		TIM2->SR = ~TIM_FLAG_CC3;
		disable_phase(LEFT_MOTOR, 2);
	}
}

void TIM5_IRQHandler()
{
	if(TIM5->SR & TIM_FLAG_Update)
	{
		TIM5->SR = ~TIM_FLAG_Update;
		TIM5->CCR1 = car.motor[RIGHT_MOTOR].pwm[0];
		TIM5->CCR2 = car.motor[RIGHT_MOTOR].pwm[1];
		TIM5->CCR3 = car.motor[RIGHT_MOTOR].pwm[2];
//SET_PIN(DEBUG_GPIO, DEBUG_PIN);


		enable_phase(RIGHT_MOTOR, 0);
		enable_phase(RIGHT_MOTOR, 1);
		enable_phase(RIGHT_MOTOR, 2);
	}
	
	if(TIM5->SR & TIM_FLAG_CC1)
	{
		TIM5->SR = ~TIM_FLAG_CC1;
//CLEAR_PIN(DEBUG_GPIO, DEBUG_PIN);
		disable_phase(RIGHT_MOTOR, 0);
	}

	if(TIM5->SR & TIM_FLAG_CC2)
	{
		TIM5->SR = ~TIM_FLAG_CC2;
		disable_phase(RIGHT_MOTOR, 1);
	}

	if(TIM5->SR & TIM_FLAG_CC3)
	{
		TIM5->SR = ~TIM_FLAG_CC3;
		disable_phase(RIGHT_MOTOR, 2);
	}
}


void USART6_IRQHandler(void)
{
	unsigned char c = USART6->DR;
	uart.input = c;
	uart.got_input = 1;
}

uint16_t get_chksum(uint8_t *buffer, uint8_t size)
{
	uint8_t i;
	uint16_t result = 0;
	uint16_t result2;

	size /= 2;
	for(i = 0; i < size; i++)
	{
		uint16_t prev_result = result;
// Not sure if word aligned
		uint16_t value = (buffer[0]) | (buffer[1] << 8);
		result += value;
// Carry bit
		if(result < prev_result) result++;
		buffer += 2;
	}

	result2 = (result & 0xff) << 8;
	result2 |= (result & 0xff00) >> 8;
	return result2;
}

void handle_radio()
{
	if(radio.packet[0] != SYNC_CODE) return;

	uint16_t chksum = get_chksum(radio.packet, PACKET_SIZE - 2);

	if((chksum & 0xff) == radio.packet[PACKET_SIZE - 2] &&
		((chksum >> 8) & 0xff) == radio.packet[PACKET_SIZE - 1])
	{
// packet good
		if(!imu.need_gyro_center)
		{
			car.led_counter++;
			if(car.led_counter >= LED_DELAY2)
			{
				TOGGLE_PIN(LED_GPIO, LED_PIN);
				car.led_counter = 0;
			}
		}

		car.throttle_reverse = radio.packet[1] & 0x1;
		car.throttle = radio.packet[2] | (radio.packet[3] << 8);
		car.steering = radio.packet[4] | (radio.packet[5] << 8);
		motor_t *left_motor = &car.motor[LEFT_MOTOR];
		motor_t *right_motor = &car.motor[RIGHT_MOTOR];


// begin gyro calibration		
		if(car.throttle >= THROTTLE_MAX && 
			!imu.have_gyro_center && 
			!imu.need_gyro_center)
		{
			imu.need_gyro_center = 1;
		}
		
		if(imu.have_gyro_center)
		{
			DISABLE_INTERRUPTS

// begin acceleration
			if(car.throttle > THROTTLE_MIN && 
				car.mode == OFF)
			{
//				car.mode = STEPPER;
//				car.period = MAX_PERIOD;
//				left_motor->phase = 0;
//				right_motor->phase = 0;

				imu.current_heading = 0;
				reset_pid(&car.heading_pid);

// the only way to get straight driving is to start at full power
				car.mode = COMMUTATING;
				car.period = MAX_PERIOD;
				left_motor->power = car.operating_power;
				right_motor->power = car.operating_power;
				left_motor->commutations = 0;
				right_motor->commutations = 0;
		 		write_motors();
			}
			else
			if(car.throttle <= THROTTLE_MIN &&
				car.throttle2 > THROTTLE_MIN)
			{
				car.mode = OFF;
				left_motor->power = 0;
				right_motor->power = 0;
			}
			else
			if(car.throttle > THROTTLE_MIN)
			{
// direction changed
				if(car.throttle_reverse2 != car.throttle_reverse)
				{
					car.mode = OFF;
				}
			}
			
			ENABLE_INTERRUPTS
			car.throttle2 = car.throttle;
			car.throttle_reverse2 = car.throttle_reverse;
		}
//TRACE2
//print_text("mode=");
//print_number(car.mode);
//print_text("period=");
//print_number(car.period);
//print_text("power=");
//print_number_nospace(left_motor->power);
//print_text(",");
//print_number(right_motor->power);
	}
}




void init_motors()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;

	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	motor_t *left_motor = &car.motor[LEFT_MOTOR];
	motor_t *right_motor = &car.motor[RIGHT_MOTOR];

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);




	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
		GPIO_Pin_1 |
		GPIO_Pin_2 |
		GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
		GPIO_Pin_1 |
		GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

// must call this before ENABLE
	ADC_RegularChannelConfig(left_motor->adc, 
		left_motor->adc_channels[0], 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(right_motor->adc, 
		right_motor->adc_channels[0], 
		1, 
		ADC_SampleTime_480Cycles);
// battery
	ADC_RegularChannelConfig(ADC3, 
		ADC_Channel_2, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(ADC1, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC2, ADC_IT_EOC, ENABLE);
	ADC_ITConfig(ADC3, ADC_IT_EOC, ENABLE);

	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
	ADC_SoftwareStartConv(ADC3);



	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

// PWM pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | 
		GPIO_Pin_4 | 
		GPIO_Pin_5 | 
		GPIO_Pin_6 |
		GPIO_Pin_7;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | 
		GPIO_Pin_5;
	GPIO_ResetBits(GPIOC, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | 
		GPIO_Pin_1 | 
		GPIO_Pin_2 | 
		GPIO_Pin_12 | 
		GPIO_Pin_13;
	GPIO_ResetBits(GPIOB, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOB, &GPIO_InitStructure);

// all motors start as off
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = MAX_PERIOD;
	

// phase timers
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = MAX_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = PHASE_PRESCALE;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_Cmd(TIM1, ENABLE);




// PWM timers
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
// this determines the resolution of the back EMF sampling
// if it's too high, glitches occur
// if it's too low, commutation is less accurate
// 1khz
//	TIM_TimeBaseStructure.TIM_Prescaler = 80;
// 8khz
//	TIM_TimeBaseStructure.TIM_Prescaler = 10;
// 10khz
//	TIM_TimeBaseStructure.TIM_Prescaler = 8;
// 16khz
	TIM_TimeBaseStructure.TIM_Prescaler = 4;
	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

// all motors start as off
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
  	TIM_OC1Init(TIM5, &TIM_OCInitStructure);
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);
	
  	TIM_Cmd(TIM5, ENABLE);
//  	TIM_Cmd(TIM2, ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM1, 
		TIM_IT_CC1, 
		ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, 
		TIM_IT_Update | 
			TIM_IT_CC1 |
			TIM_IT_CC2 |
			TIM_IT_CC3, 
		ENABLE);


 	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM5, 
		TIM_IT_Update | 
			TIM_IT_CC1 |
			TIM_IT_CC2 |
			TIM_IT_CC3, 
		ENABLE);

	write_motors();

}


int main(void)
{
	int i, j;


	init_linux();
	bzero(&car, sizeof(car_t));

	car.steering = STEERING_MID;
	car.stepper_power = MAX_PWM * 50 / 100;
//	car.stepper_power = MAX_PWM;
//	car.operating_power0 = MAX_PWM;
//	car.operating_power = MAX_PWM * 93 / 100;
	car.operating_power = MAX_PWM;

	car.mode = OFF;
	car.period = MAX_PERIOD;
	motor_t *left_motor = &car.motor[LEFT_MOTOR];
	motor_t *right_motor = &car.motor[RIGHT_MOTOR];
	
	left_motor->commutation_direction = -1;
	left_motor->stepper_direction = -1;
	left_motor->turn_sign = 1;
	left_motor->pwm_timer = TIM2;
	left_motor->adc = ADC2;
	left_motor->sensor_order = 0;
	left_motor->threshold = 2100;
	left_motor->adc_channels[0] = left_sensor_table[left_motor->sensor_order][0];
	left_motor->adc_channels[1] = left_sensor_table[left_motor->sensor_order][1];
	left_motor->adc_channels[2] = left_sensor_table[left_motor->sensor_order][2];
	left_motor->in_pin[0] = GPIO_Pin_1;
	left_motor->in_gpio[0] = GPIOB;
	left_motor->in_pin[1] = GPIO_Pin_2;
	left_motor->in_gpio[1] = GPIOB;
	left_motor->in_pin[2] = GPIO_Pin_13;
	left_motor->in_gpio[2] = GPIOB;
	left_motor->en_pin[0] = GPIO_Pin_0;
	left_motor->en_gpio[0] = GPIOB;
	left_motor->en_pin[1] = GPIO_Pin_12;
	left_motor->en_gpio[1] = GPIOB;
	left_motor->en_pin[2] = GPIO_Pin_5;
	left_motor->en_gpio[2] = GPIOC;

    right_motor->commutation_direction = -1;
    right_motor->stepper_direction = -1;
    right_motor->turn_sign = -1;
    right_motor->pwm_timer = TIM5;
    right_motor->adc = ADC1;
	right_motor->sensor_order = 1;
	right_motor->threshold = 2070;
	right_motor->adc_channels[0] = right_sensor_table[right_motor->sensor_order][0];
	right_motor->adc_channels[1] = right_sensor_table[right_motor->sensor_order][1];
	right_motor->adc_channels[2] = right_sensor_table[right_motor->sensor_order][2];
	right_motor->in_pin[0] = GPIO_Pin_5;
	right_motor->in_gpio[0] = GPIOA;
	right_motor->in_pin[1] = GPIO_Pin_6;
	right_motor->in_gpio[1] = GPIOA;
	right_motor->in_pin[2] = GPIO_Pin_4;
	right_motor->in_gpio[2] = GPIOC;
	right_motor->en_pin[0] = GPIO_Pin_4;
	right_motor->en_gpio[0] = GPIOA;
	right_motor->en_pin[1] = GPIO_Pin_7;
	right_motor->en_gpio[1] = GPIOA;
	right_motor->en_pin[2] = GPIO_Pin_3;
	right_motor->en_gpio[2] = GPIOA;

	for(i = 0; i < SENSORS; i++)
	{
		for(j = 0; j < MOTORS; j++)
		{
			car.motor[j].adc_min[i] = 65536;
			car.motor[j].adc_max[i] = 0;
		}
	}

//	car.test_motors = 1;
	if(car.test_motors)
	{
		car.motor[LEFT_MOTOR].power = MAX_PWM;
		car.motor[RIGHT_MOTOR].power = MAX_PWM;
	}

/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC |
			RCC_AHB1Periph_GPIOD |
			RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_CCMDATARAMEN, 
		ENABLE);

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);


// general purpose timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 50;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);




	init_uart();
	
 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

 	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM10, 
		TIM_IT_Update, 
		ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	SET_PIN(LED_GPIO, LED_PIN);

// debug pin
	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
	GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);



	print_text("Welcome to brushless direct drive rover\n");
	print_text("left sensors: ");
	print_number(left_motor->sensor_order);
	print_text("right sensors: ");
	print_number(right_motor->sensor_order);
	print_lf();


	flush_uart();
	
	init_motors();
	init_cc1101();
	cc1101_receiver();
	init_imu(&imu);

// DEBUG
// bypass calibration for testing
//	imu.have_gyro_center = 1;

// heading error -> PWM
	init_pid(&car.heading_pid, 
//		2000, // P gain
		4000, // P gain
		1, // I gain	
//		40000, // D gain
		80000, // D gain
		MAX_PWM * 15 / 100, // I limit
		MAX_PWM * 15 / 100); // O limit

//	test_motors();
	
// test floating point
//	float x = 0.12345f;
//	float y = tanf(x);
//    print_float(x);
//    print_float(y);
	

	while(1)
	{

		handle_uart();
//		handle_timer1();
		if(radio.got_packet)
		{
			radio.got_packet = 0;
			handle_radio();
		}

		if(car.got_battery) handle_battery();
		HANDLE_IMU(imu);
		if(imu.got_ahrs) handle_controls();

// debugging commands
// must disable when UART is not plugged in
		if(0 && uart_got_input())
		{
			unsigned char c = uart_get_input();
			int do_status = 0;
			
			switch(c)
			{
				case ']':
					left_motor->sensor_order++;
					if(left_motor->sensor_order >= TOTAL_SENSOR_ORDERS) left_motor->sensor_order = 0;
					do_status = 1;
					break;
				
				case '[':
					left_motor->sensor_order--;
					if(left_motor->sensor_order < 0) left_motor->sensor_order = TOTAL_SENSOR_ORDERS - 1;
					do_status = 1;
					break;
				
				case '{':
					right_motor->sensor_order++;
					if(right_motor->sensor_order >= TOTAL_SENSOR_ORDERS) right_motor->sensor_order = 0;
					do_status = 1;
					break;
				
				case '}':
					right_motor->sensor_order--;
					if(right_motor->sensor_order < 0) right_motor->sensor_order = TOTAL_SENSOR_ORDERS - 1;
					do_status = 1;
					break;
				
				case '+':
  				case '=':
					car.operating_power++;
					left_motor->power++;
					right_motor->power++;
					write_motors();
					do_status = 1;
					break;

				case '-':
					car.operating_power--;
					left_motor->power--;
					right_motor->power--;
					write_motors();
					do_status = 1;
					break;
			}
			
			if(do_status)
  			{
				left_motor->adc_channels[0] = left_sensor_table[left_motor->sensor_order][0];
				left_motor->adc_channels[1] = left_sensor_table[left_motor->sensor_order][1];
				left_motor->adc_channels[2] = left_sensor_table[left_motor->sensor_order][2];

				right_motor->adc_channels[0] = right_sensor_table[right_motor->sensor_order][0];
				right_motor->adc_channels[1] = right_sensor_table[right_motor->sensor_order][1];
				right_motor->adc_channels[2] = right_sensor_table[right_motor->sensor_order][2];

  				TRACE2
//  			print_number(left_motor->power);
//  			print_number(right_motor->power);
				print_text("power=");
  				print_number(car.operating_power);
				print_text("left sensors=");
	  			print_number(left_motor->sensor_order);
				print_text("right sensors=");
  				print_number(right_motor->sensor_order);
			}
		}
		

/*
 * 		if(uart_got_input())
 * 		{
 * 			unsigned char c = uart_get_input();
 * 			int do_status = 0;
 * 
 * 			if(c == '+' ||
 * 				c == '=')
 * 			{
 * 				left_motor->pwm[0]++;
 * 				left_motor->pwm[1]++;
 * 				left_motor->pwm[2]++;
 * 				do_status = 1;
 * 			}
 * 			else
 * 			if(c == '-')
 * 			{
 * 				left_motor->pwm[0]--;
 * 				left_motor->pwm[1]--;
 * 				left_motor->pwm[2]--;
 * 				do_status = 1;
 * 			}
 * 
 * 			if(do_status)
 * 			{
 * 				TRACE2
 * 				print_number(left_motor->pwm[0]);
 * 				print_number(left_motor->pwm[1]);
 * 				print_number(left_motor->pwm[2]);
 * 				do_status = 0;
 * 			}
 * 		}
 */


	}
	
		
}





