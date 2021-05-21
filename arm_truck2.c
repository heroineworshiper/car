/*
 * STM32 Controller for direct drive truck
 * Copyright (C) 2012-2021 Adam Williams <broadcast at earthling dot net>
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

// make truck2.bin;uart_programmer truck2.bin

// pass bluetooth to the debug port to configure the device by enabling 
// BLUETOOTH_PASSTHROUGH
// useful configuration commands:
// at+version
// AT+VERSION
// at+nametruck
// AT+NAMEtruck
// AT+NAMEcar
// at+baud8
// AT+BAUD8

// new devices start at 9600 baud.  Be sure to set the initial baud rate 
// to 9600, then 115200 after configuration.

// A delay in the terminal program is required to paste text in 
// without overflowing the passthrough buffer,
// but the entire command must be sent in under a second, so adjust the terminal
// program's delay.

// A0/TIM5 CH1 - L motor IN1
// A1/TIM5 CH2 - L motor IN3
// A2/TIM5 CH3 - L motor IN2
// A6/TIM3 CH1 - R motor IN1
// A7/TIM3 CH2 - R motor IN3
// B0/TIM3 CH3 - R motor IN2

// TIM10 - system tick
// TIM2 - steering PWM
// TIM1 - hall effect sensor tick
// UART5 - 900Mhz radio
// UART1 - bluetooth





#include "arm_truck2.h"
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



#define SYNC_CODE 0xe5


// start of persistent settings
#define SETTINGS_START 0x0800c000
// end of settings
#define SETTINGS_END 0x08010000
// size of settings block
#define SETTINGS_SIZE 256
// Magic number for settings
#define SETTINGS_MAGIC 0x10291976


// memory map page 54:
// 0x08000000: sector 0 bootloader 48k
// 0x0800c000: sector 3 settings 720k
// 0x080c0000: sector 10 program 256k



// commands we get from the phone
#define GET_STATUS 0
#define RESET_COMMAND 1
#define NEW_CONFIG 2

// packets we send to the phone
#define STATUS_PACKET 0

// binary steering range
#define FAST_LEFT 1
#define SLOW_LEFT 2
#define STEERING_MID 0x0
#define SLOW_RIGHT 3
#define FAST_RIGHT 4


#define BATTERY_OVERSAMPLE 10000
#define GYRO_OVERSAMPLE 64
#define GYRO_CENTER_TOTAL (NAV_HZ)
#define CURRENT_OVERSAMPLE 5000

// radio timeout
#define TIMEOUT_RELOAD (TIMER_HZ * 1)
#define STEERING_RELOAD (10 * PWM_HZ)

// timer for LED flashing
#define LED_DELAY (NAV_HZ / 2)
// ADC for 0 current
#define CURRENT_BASE 120.0f
// scale factor for current
#define CURRENT_SCALE 630.0f

#define CLOCKSPEED 168000000
#define SERVO_PWM_PERIOD (CLOCKSPEED / 2 / PWM_HZ)

// 2ms
#define MAX_PWM 168000
// 1ms
#define MIN_PWM 84000

#define MAX_PERIOD 65535

//#define DEBUG_I2C
#ifdef DEBUG_I2C
#define I2C_SIZE 1024
int i2c_offset = 0;
uint8_t i2c_capture[I2C_SIZE];
int capture_i2c = 0;
#endif



truck_t truck;

int get_motor_angle(int motor, int hall);


// created by tables.c
const uint8_t sin_table[] = 
{
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

#define WAVEFORM_SIZE (sizeof(sin_table) / sizeof(uint8_t))
#define MAX_SIN 255
#define CALCULATE_WAVEFORM(x) ((int)sin_table[x] * truck.power / MAX_SIN)
#define ABS(x) ((x) > 0 ? (x) : (-(x)))
#define V_TO_MPH(v) ((v) * 10 * 3600 * 3.14 * 107 / 360 / 1609363)





void write_pwm();
void write_motors();



float init_pid(pid_t *pid, 
	float p_gain, 
	float i_gain,
	float d_gain,
	float d2_gain,
	float p_limit,
	float i_limit,
	float d_limit)
{
	pid->i_result = 0;
	pid->p_gain = p_gain;
	pid->i_gain = i_gain;
	pid->d_gain = d_gain;
	pid->d2_gain = d2_gain;
	pid->p_limit = p_limit;
	pid->i_limit = i_limit;
	pid->d_limit = d_limit;
}

void reset_pid(pid_t *pid)
{
	pid->error_accum = 0;
	pid->i_result = 0;
	pid->counter = 0;
	pid->ignore_i = 0;
	pid->result = 0;
}

void dump_pid(pid_t *pid)
{
	print_float(pid->p_gain);
	print_float(pid->i_gain);
	print_float(pid->d_gain);
	print_float(pid->d2_gain);
	print_lf();
}

int read_pid(pid_t *pid, const unsigned char *buffer, int offset)
{
	float p = READ_FLOAT32(buffer, offset);
	float i = READ_FLOAT32(buffer, offset);
	float d = READ_FLOAT32(buffer, offset);
	float d2 = READ_FLOAT32(buffer, offset);
	float p_limit = 100;
	float i_limit = 100;
	float d_limit = 100;
	init_pid(pid, 
		p, // P gain
		i, // I gain	
		d, // D gain
		d2, // D2 gain
		p_limit, 
		i_limit, 
		d_limit);
	reset_pid(pid);
	return offset;
}

float do_pid(pid_t *pid, float p_error, float d_error, float d2_error)
{
	pid->p_input = p_error;
	pid->d_input = d_error;
	pid->d2_input = d2_error;


	pid->p_result = p_error * pid->p_gain;
	CLAMP(pid->p_result, -pid->p_limit, pid->p_limit);
	
	pid->d_result = d_error * pid->d_gain;
	CLAMP(pid->d_result, -pid->d_limit, pid->d_limit);

	pid->d2_result = d2_error * pid->d2_gain;
	CLAMP(pid->d2_result, -pid->d_limit, pid->d_limit);

	pid->error_accum += p_error;
	pid->counter++;
	if(pid->counter >= truck.pid_downsample * PWM_HZ / PWM_BASE)
	{
// average of all errors
		pid->error_accum /= pid->counter;

// proportional I factor
		if(!pid->ignore_i)
		{
			pid->i_result += pid->error_accum * pid->i_gain;
		}
		else
		{
			pid->ignore_i = 0;
		}

		
		CLAMP(pid->i_result, -pid->i_limit, pid->i_limit);
		pid->counter = 0;
		pid->error_accum = 0;
	}
	

	pid->result = pid->p_result + 
		pid->i_result + 
		pid->d_result + 
		pid->d2_result;



	CLAMP(pid->result, -100, 100);
	return pid->result;
}




void init_filter(filter_t *ptr, float value, float bandwidth)
{
	int i;
	ptr->bandwidth = bandwidth;
	for(i = 0; i < ORDER; i++)
	{
		ptr->prev_output[i] = value;
		ptr->prev_input[i] = value;
	}
}

void reset_filter(filter_t *ptr)
{
    int i;
	for(i = 0; i < ORDER; i++)
	{
        ptr->prev_output[i] = 0;
        ptr->prev_input[i] = 0;
    }
}

float do_highpass(filter_t *ptr, float value)
{
	int i;
	float result = value;
	for(i = 0; i < ORDER; i++)
	{
		result = ptr->bandwidth * (ptr->prev_output[i] + value - ptr->prev_input[i]);
		ptr->prev_input[i] = value;
		ptr->prev_output[i] = result;
		value = result;
	}
	
	return result;
}

float do_lowpass(filter_t *ptr, float value)
{
	int i;
	float result = value;
	for(i = 0; i < ORDER; i++)
	{
		result = ptr->prev_output[i] + ptr->bandwidth * (value - ptr->prev_output[i]);
		ptr->prev_output[i] = result;
		value = result;
	}
	
	return result;
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




void handle_radio_packet(unsigned char *ptr)
{
    truck.stick_packets++;

// packet good
	TOGGLE_PIN(LED_GPIO, GREEN_LED);

// protect radio_timeout
	DISABLE_INTERRUPTS
	truck.radio_timeout = TIMEOUT_RELOAD;
	ENABLE_INTERRUPTS

	truck.throttle = ptr[1];
	truck.steering = ptr[0];
    truck.speed_offset = ptr[2];
    if(ptr[2] & 0x80)
    {
        truck.speed_offset = truck.speed_offset - 0x100;
    }

#ifdef BINARY_STEERING
// convert to a binary steering value
    truck.binary_steering = STEERING_MID;
    if(truck.steering > truck.remote_steering_mid + truck.remote_steering_deadband)
    {
        if(truck.steering >= truck.remote_steering_max)
        {
            truck.binary_steering = FAST_RIGHT;
        }
        else
        {
            truck.binary_steering = SLOW_RIGHT;
        }
    }
    else
    if(truck.steering < truck.remote_steering_mid - truck.remote_steering_deadband)
    {
        if(truck.steering <= truck.remote_steering_min)
        {
            truck.binary_steering = FAST_LEFT;
        }
        else
        {
            truck.binary_steering = SLOW_LEFT;
        }
    }

//TRACE2
//print_number(truck.binary_steering);


#endif // BINARY_STEERING

// TRACE2
// print_number(truck.throttle);
// print_number(truck.steering);
// print_number(truck.speed_offset);



// DEBUG
// send throttle to steering PWM to test an ESC
//     if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband)
//     {
//         truck.steering_pwm = (MAX_PWM + MIN_PWM) / 2 +
//             (MAX_PWM - MIN_PWM) / 2 * 
//             (truck.throttle - truck.remote_throttle_mid - truck.remote_throttle_deadband) /
//             (truck.remote_throttle_max - truck.remote_throttle_mid - truck.remote_throttle_deadband);
//     }
//     else
//     if(truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
//     {
//         truck.steering_pwm = (MAX_PWM + MIN_PWM) / 2 -
//             (MAX_PWM - MIN_PWM) / 2 * 
//             (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.throttle) /
//             (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.remote_throttle_min);
//     }
//     else
//     {
//         truck.steering_pwm = (MAX_PWM + MIN_PWM) / 2;
//     }
//     write_pwm();

}


void dump_config()
{
	TRACE2
	print_text("\nmid_steering100=");
	print_number(truck.mid_steering100);
	print_text("\nmax_steering100=");
	print_number(truck.max_steering100);
	print_text("\nmin_steering100=");
	print_number(truck.min_steering100);

	print_text("\nthrottle_base100=");
	print_number(truck.throttle_base100);
	print_text("\nthrottle_reverse_base100=");
	print_number(truck.throttle_reverse_base100);
	print_text("\nmin_throttle_fwd100=");
	print_number(truck.min_throttle_fwd100);
	print_text("\nmin_throttle_reverse100=");
	print_number(truck.min_throttle_reverse100);

	print_text("\nrpm_dv_size=");
	print_float(truck.rpm_dv_size);
	print_text("\ngyro_center_max=");
	print_number(truck.gyro_center_max);
	print_text("\nmax_gyro_drift=");
	print_float(truck.max_gyro_drift);
	print_text("\nangle_to_gyro=");
	print_number(truck.angle_to_gyro);
	print_text("\ngyro_bandwidth=");
	print_number((int)(truck.imu.gyro_bandwidth * 255));
	print_text("\nd_bandwidth=");
	print_number(truck.d_bandwidth);

	print_text("\npid_downsample=");
	print_number(truck.pid_downsample);
	print_text("\nmax_rpm=");
	print_float(truck.max_rpm);
	print_text("\nmin_rpm=");
	print_float(truck.min_rpm);
	print_text("\nmax_reverse_rpm=");
	print_float(truck.max_reverse_rpm);
	print_text("\nmin_reverse_rpm=");
	print_float(truck.min_reverse_rpm);
	print_text("\ndiameter=");
	print_number(truck.diameter);

	


	print_text("\nbattery_analog=");
	print_number(truck.battery_analog);
	print_text("\nbattery_v0=");
	print_float(truck.battery_v0);
	print_text("\nmin_steering_step=");
	print_float(TO_DEG(truck.min_steering_step));
	print_text("\nmax_steering_step=");
	print_float(TO_DEG(truck.max_steering_step));
	print_text("\nsteering_overshoot=");
	print_float(TO_DEG(truck.steering_overshoot));

	print_text("\nSTEERING PID=");
	dump_pid(&truck.heading_pid);

	print_text("RPM PID=");
	dump_pid(&truck.rpm_pid);
}

int read_config_packet(const unsigned char *buffer)
{
	int offset = 0;

    truck.min_throttle_fwd100 = buffer[offset++];
    truck.min_throttle_reverse100 = buffer[offset++];
    truck.throttle_base100 = buffer[offset++];
    truck.throttle_reverse_base100 = buffer[offset++];

	truck.mid_steering100 = buffer[offset++];
	truck.max_steering100 = buffer[offset++];
	truck.min_steering100 = buffer[offset++];
	truck.rpm_dv_size = buffer[offset++];

	truck.gyro_center_max = READ_UINT16(buffer, offset);
	truck.max_gyro_drift = (float)READ_UINT16(buffer, offset) / 256;
	truck.angle_to_gyro = READ_INT16(buffer, offset);
	truck.imu.gyro_bandwidth = (float)buffer[offset++] / 0xff;
	truck.d_bandwidth = (float)buffer[offset++];


	truck.pid_downsample = READ_UINT16(buffer, offset);
	
	
	truck.battery_analog = READ_UINT16(buffer, offset);

	truck.max_rpm = READ_UINT16(buffer, offset);
	truck.max_reverse_rpm = READ_UINT16(buffer, offset);
	truck.min_reverse_rpm = READ_UINT16(buffer, offset);
	truck.min_rpm = READ_UINT16(buffer, offset);
	truck.diameter = buffer[offset++];
	
	truck.battery_v0 = READ_FLOAT32(buffer, offset);
	truck.min_steering_step = READ_FLOAT32(buffer, offset);
	truck.max_steering_step = READ_FLOAT32(buffer, offset);
	truck.steering_overshoot = READ_FLOAT32(buffer, offset);
	
	
	offset = read_pid(&truck.heading_pid, buffer, offset);
	offset = read_pid(&truck.rpm_pid, buffer, offset);
	
	init_filter(&truck.d_filter, 0, truck.d_bandwidth / 100);
	
	resize_derivative(&truck.rpm_dv, truck.rpm_dv_size);

	truck.mid_steering_pwm = MIN_PWM + 
			(MAX_PWM - MIN_PWM) * 
			truck.mid_steering100 /
			100;
	truck.min_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.min_steering100 /
			100;
	truck.max_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.max_steering100 / 
			100;

/*
 * TRACE2
 * print_text("size of config packet=");
 * print_number(offset);
 */



	return offset;
}

uint32_t next_config_address()
{
	uint32_t address = SETTINGS_START;
	while(address < SETTINGS_END)
	{
		unsigned char *ptr = (unsigned char *)address;
		if((ptr[0] == 0xff) &&
			(ptr[1] == 0xff) &&
			(ptr[2] == 0xff) &&
			(ptr[3] == 0xff))
		{
			return address;
		}
		address += SETTINGS_SIZE;
	}
	return address;
}

void save_config(unsigned char *buffer, int bytes)
{
    int attempts = 0;
    int max_attempts = 3;

    TRACE2
    for(attempts = 0; attempts < max_attempts; attempts++)
    {
// try writing it
   	    USART_Cmd(UART5, DISABLE);
   	    USART_Cmd(USART1, DISABLE);
   	    USART_Cmd(USART6, DISABLE);

	    FLASH_Unlock();
	    FLASH_ClearFlag(FLASH_FLAG_EOP | 
		    FLASH_FLAG_OPERR | 
		    FLASH_FLAG_WRPERR | 
    	    FLASH_FLAG_PGAERR | 
		    FLASH_FLAG_PGPERR |
		    FLASH_FLAG_PGSERR); 


// get next address to write settings
	    uint32_t address = next_config_address();
	    if(address == SETTINGS_END)
	    {
		    address = SETTINGS_START;
    	    if (FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3) != FLASH_COMPLETE)
		    {
		    }
	    }

        uint32_t address2 = address;
        const unsigned char *ptr = (unsigned char*)address;
	    const unsigned char start_code[] = 
	    {
		    (SETTINGS_MAGIC >> 24) & 0xff,
		    (SETTINGS_MAGIC >> 16) & 0xff,
		    (SETTINGS_MAGIC >> 8) & 0xff,
		    SETTINGS_MAGIC & 0xff,
	    };


	    /* Clear pending flags (if any) */  
	    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
            FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
	    if(FLASH_ProgramWord(address, *(int*)(start_code)) != FLASH_COMPLETE)
	    {
	    }

	    address += 4;


    // Align on 4 bytes
	    while(bytes % 4) buffer[bytes++] = 0;
	    int i;
	    for(i = 0; i < bytes; i += 4)
	    {
		    if (FLASH_ProgramWord(address + i, *(int*)(buffer + i)) != FLASH_COMPLETE)
		    {
		    }
	    }


	    FLASH_Lock(); 


   	    USART_Cmd(USART6, ENABLE);
   	    USART_Cmd(USART1, ENABLE);
   	    USART_Cmd(UART5, ENABLE);

// verify it
        int pass = 1;
        if(start_code[0] != ptr[0] ||
            start_code[1] != ptr[1] ||
            start_code[2] != ptr[2] ||
            start_code[3] != ptr[3])
        {
            pass = 0;
        }

        for(i = 0; i < bytes && pass; i++)
        {
            if(ptr[4 + i] != buffer[i])
            {
                pass = 0;
            }
        }

        if(pass)
        {
	        TRACE2
            print_text("address=");
            print_hex(address2);
	        print_text("Saved flash config\n");
            break;
        }
        else
        if(attempts < max_attempts - 1)
        {
	        TRACE2
            print_text("address=");
            print_hex(address2);
	        print_text("Flash verify failed.\n");
        }
        else
        {
	        TRACE2
            print_text("address=");
            print_hex(address2);
	        print_text("All flash verifies failed.  Giving up & going to a movie.\n");
        }
    }
}

// called during bootup only
void load_config()
{
// Load settings from flash
	uint32_t address = next_config_address();
	if(address > SETTINGS_START) address -= SETTINGS_SIZE;

	const unsigned char *buffer = (unsigned char*)address;
	if((buffer[0] == ((SETTINGS_MAGIC >> 24) & 0xff)) &&
		(buffer[1] == ((SETTINGS_MAGIC >> 16) & 0xff)) &&
		(buffer[2] == ((SETTINGS_MAGIC >> 8) & 0xff)) &&
		(buffer[3] == (SETTINGS_MAGIC & 0xff)))
	{
		TRACE2
		print_text("Loading flash config\n");
		read_config_packet(buffer + 4);
	}
}


#ifdef BLUETOOTH_PASSTHROUGH
static void bluetooth_passthrough()
{
	send_uart_binary(&truck.bluetooth.data, 1);
}
#endif // BLUETOOTH_PASSTHROUGH

static void handle_beacon()
{
	unsigned char *receive_buf = truck.bluetooth.receive_buf;
	int receive_size = truck.bluetooth.receive_size;
	uint16_t chksum = get_chksum(receive_buf, 
		receive_size - 2);
//TRACE2
	if((chksum & 0xff) == receive_buf[receive_size - 2] &&
		((chksum >> 8) & 0xff) == receive_buf[receive_size - 1])
	{
        truck.bluetooth_packets++;
    
		switch(receive_buf[6])
		{
// battery voltage
			case GET_STATUS:
			{
// get fully manual controls
				if(receive_buf[8] == 1)
				{
					DISABLE_INTERRUPTS
					truck.bt_throttle = receive_buf[9];
					if(truck.bt_throttle & 0x80) truck.bt_throttle = -256 + truck.bt_throttle;
					truck.bt_steering = receive_buf[10];
					if(truck.bt_steering & 0x80) truck.bt_steering = -256 + truck.bt_steering;
					truck.have_bt_controls = 1;
					truck.bt_timeout = TIMEOUT_RELOAD;
					ENABLE_INTERRUPTS

                    TOGGLE_PIN(LED_GPIO, GREEN_LED);

/*
 * TRACE2
 * print_text("bt_throttle=");
 * print_number(truck.bt_throttle);
 * print_text(" bt_steering=");
 * print_number(truck.bt_steering);
 */

				}
				else
// nested radio packet
				if(receive_buf[8] == 2)
				{
					handle_radio_packet(receive_buf + 10);
				}
				else
				{
					DISABLE_INTERRUPTS
					truck.have_bt_controls = 0;
					ENABLE_INTERRUPTS
				}

// create return packet
				int offset = 0;
				int offset2 = 4;
                unsigned char *buf = truck.bluetooth.send_buf;
				buf[offset++] = 0xff;
				buf[offset++] = 0x2d;
				buf[offset++] = 0xd4;
				buf[offset++] = 0xe5;
// size
				WRITE_INT16(buf, offset, 0);
// battery response
				buf[offset++] = STATUS_PACKET;
				buf[offset++] = 0;
				
				WRITE_INT32(buf, offset, truck.battery);
				WRITE_FLOAT32(buf, offset, truck.battery_voltage);
				WRITE_INT16(buf, offset, (int)truck.imu.gyro_z_center);
				WRITE_INT16(buf, offset, truck.imu.gyro_z_max - truck.imu.gyro_z_min);
				WRITE_INT16(buf, offset, truck.rpm);
        		WRITE_FLOAT32(buf, offset, truck.current_heading);

                WRITE_INT32(buf, offset, truck.throttle_avg);
                WRITE_INT16(buf, offset, truck.bluetooth_packets2);
                WRITE_INT16(buf, offset, truck.stick_packets2);
                buf[offset++] = truck.throttle_updated;
                truck.throttle_updated = 0;

                buf[offset++] = truck.steering;
                buf[offset++] = truck.throttle;


// write the correct size
				WRITE_INT16(buf, offset2, offset + 2);

// write the chksum
				chksum = get_chksum(buf, offset);
				WRITE_INT16(buf, offset, chksum);

// send it
				truck.bluetooth.send_offset = 0;
				truck.bluetooth.send_size = offset;
				
//TRACE2
//print_buffer(buf, truck.bluetooth.send_size);

// TRACE2
// print_text("heading ");
// print_float(truck.current_heading);
// print_text("feedback ");
// print_float(truck.heading_pid.result);

				break;
			}

// reset gyro
			case RESET_COMMAND:
				DISABLE_INTERRUPTS
				truck.have_gyro_center = 0;
				truck.gyro_center_count = 0;
				truck.gyro_center = 0;
				truck.gyro_accum = 0;
				truck.gyro_min = 0;
				truck.gyro_max = 0;
				truck.current_heading = 0;
// red
				SET_PIN(LED_GPIO, RED_LED);
				ENABLE_INTERRUPTS
				break;

// config file
			case NEW_CONFIG:
			{
				int offset = 8;
// receive_buf is overwritten if this routine takes too long

				int bytes = read_config_packet(receive_buf + offset);
				int need_save = receive_buf[7];
//TRACE2
//print_buffer(truck.bluetooth.receive_buf, truck.bluetooth.receive_size);

				if(need_save)
				{
					save_config(receive_buf + offset,
						bytes);

					CLEAR_PIN(LED_GPIO, RED_LED);
					int i;
					int prev_timer_h = truck.tick;
					int toggle = 0;
// stop the feedback loop
					truck.writing_settings = 1;

// wiggle the wheels & flash the LED if not driving
                    if(truck.throttle <= truck.remote_throttle_mid + truck.remote_throttle_deadband &&
                        truck.throttle >= truck.remote_throttle_mid - truck.remote_throttle_deadband)
                    {
					    for(i = 0; i < 2; i++)
					    {
						    if(toggle == 0)
						    {
							    truck.steering_pwm = truck.mid_steering_pwm - truck.max_steering_magnitude;
							    write_pwm();
						    }
						    else
						    {
							    truck.steering_pwm = truck.mid_steering_pwm + truck.max_steering_magnitude;
							    write_pwm();
						    }
						    toggle ^= 1;

						    while(1)
						    {
							    DISABLE_INTERRUPTS
							    int time_difference = truck.tick - prev_timer_h;
							    ENABLE_INTERRUPTS
	    // seems required because of a compiler error
							    flush_uart();
							    if(time_difference >= TIMER_HZ / 2) break;
						    }

						    DISABLE_INTERRUPTS
						    prev_timer_h = truck.tick;
						    ENABLE_INTERRUPTS

						    TOGGLE_PIN(LED_GPIO, RED_LED);
					    }
                    }

					truck.writing_settings = 0;
// solid red
					SET_PIN(LED_GPIO, RED_LED);
				}

				dump_config();
				break;
			}

		}
	}
}

static void bt_get_code1();
static void bt_get_code2();

static void bt_get_data()
{
	truck.bluetooth.receive_buf[truck.bluetooth.receive_offset++] = 
		truck.bluetooth.data;
	if(truck.bluetooth.receive_offset >= truck.bluetooth.receive_size)
	{
		truck.bluetooth.got_data = 1;
		truck.bluetooth.current_function = bt_get_code1;
	}
}

static void bt_get_size()
{
	truck.bluetooth.receive_buf[truck.bluetooth.receive_offset++] = 
		truck.bluetooth.data;
	if(truck.bluetooth.receive_offset >= truck.bluetooth.receive_size)
	{
		truck.bluetooth.receive_size = truck.bluetooth.receive_buf[4] |
			(truck.bluetooth.receive_buf[5] << 8);
		if(truck.bluetooth.receive_size < BLUE_BUFSIZE)
		{
			truck.bluetooth.current_function = bt_get_data;
		}
		else
		{
			truck.bluetooth.current_function = bt_get_code1;
		}
	}
}

static void bt_get_code4()
{
	if(truck.bluetooth.data == 0xe5)
	{
		truck.bluetooth.receive_buf[0] = 0xff;
		truck.bluetooth.receive_buf[1] = 0x2d;
		truck.bluetooth.receive_buf[2] = 0xd4;
		truck.bluetooth.receive_buf[3] = 0xe5;
		truck.bluetooth.current_function = bt_get_size;
		truck.bluetooth.receive_offset = 4;
		truck.bluetooth.receive_size = 6;
	}
	else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = bt_get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = bt_get_code1;
    }
}


static void bt_get_code3()
{
	if(truck.bluetooth.data == 0xd4)
	{
    	truck.bluetooth.current_function = bt_get_code4;
	}
    else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = bt_get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = bt_get_code1;
    }
}


static void bt_get_code2()
{
	if(truck.bluetooth.data == 0x2d)
    {
		truck.bluetooth.current_function = bt_get_code3;
	}
    else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = bt_get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = bt_get_code1;
    }
}


static void bt_get_code1()
{
	if(truck.bluetooth.data == 0xff)
	{
    	truck.bluetooth.current_function = bt_get_code2;
    }
}


// The sys tick handler
void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
		
		truck.tick++;

// Update shutdown timer
		if(truck.bt_timeout > 0)
		{
			truck.bt_timeout--;
		}
		if(truck.radio_timeout > 0)
		{
			truck.radio_timeout--;
		}

// update statistics
        if(truck.tick - truck.stat_time >= TIMER_HZ)
        {
            if(truck.throttle_count > 0)
            {
                truck.throttle_avg = truck.throttle_accum / truck.throttle_count;
                truck.throttle_accum = 0;
                truck.throttle_count = 0;
                truck.throttle_updated = 1;
            }
            
            truck.bluetooth_packets2 = truck.bluetooth_packets;
            truck.stick_packets2 = truck.stick_packets;
            truck.bluetooth_packets = 0;
            truck.stick_packets = 0;
            
            truck.stat_time = truck.tick;

//             TRACE2
//             print_text("total_gyro=");
//             print_number(truck.imu.total_gyro);
//             print_text("heading=");
//             print_float(TO_DEG(truck.current_heading));


            truck.imu.total_gyro = 0;
        }

	}
}


void handle_bluetooth()
{

#ifdef BLUETOOTH_PASSTHROUGH
// if((USART3->SR & USART_FLAG_TC) != 0)
// {
//     USART3->DR = 'X';
// }
	if(uart_got_input() && (USART3->SR & USART_FLAG_TC) != 0)
	{
		unsigned char c = uart_get_input();
// print_text("sending ");
// send_uart(&c, 1);
// print_lf();

		USART3->DR = c;
	}
#else // BLUETOOTH_PASSTHROUGH

	if(truck.bluetooth.send_offset < truck.bluetooth.send_size &&
		(USART1->SR & USART_FLAG_TC) != 0)
	{
		USART1->DR = truck.bluetooth.send_buf[truck.bluetooth.send_offset++];
	}

	if(truck.bluetooth.got_data)
	{
		truck.bluetooth.got_data = 0;
		handle_beacon();
	}

#endif // !BLUETOOTH_PASSTHROUGH
}


void handle_analog()
{
// battery
	if((ADC3->SR & ADC_FLAG_EOC))
	{
		truck.battery_accum += ADC3->DR;
// start the next conversion
		ADC3->CR2 |= (uint32_t)ADC_CR2_SWSTART;
		truck.battery_count++;
		if(truck.battery_count >= BATTERY_OVERSAMPLE)
		{
			truck.battery = truck.battery_accum / truck.battery_count;



			truck.battery_accum = 0;
			truck.battery_count = 0;
			truck.battery_voltage = (float)truck.battery * 
				truck.battery_v0 / 
				truck.battery_analog;


// TRACE2
// print_number(truck.battery_accum);
// print_number(truck.battery_count);
// print_number(truck.battery);
// print_float(truck.battery_voltage);


		}
	
	}






}


void init_analog()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
// The battery
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
	ADC_Init(ADC3, &ADC_InitStructure);



	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;


	GPIO_Init(GPIOC, &GPIO_InitStructure);


// must call this before ENABLE
	ADC_RegularChannelConfig(ADC3, 
		ADC_Channel_10, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC3, ENABLE);
// start the next conversion
	ADC3->CR2 |= (uint32_t)ADC_CR2_SWSTART;


}

// steering input with power
// void auto_steering_input(float *steering_overshoot)
// {
// 
// // deadband
//     if(truck.steering <= truck.remote_steering_mid + truck.remote_steering_deadband &&
//         truck.steering >= truck.remote_steering_mid - truck.remote_steering_deadband)
//     {
// // Next steering press engages heading hold
// 		truck.auto_steering = 1;
// 		truck.need_steering_feedback = 1;
//     }
//     else
//     if(truck.steering > truck.remote_steering_mid + truck.remote_steering_deadband)
//     {
// // proportional right
// 		if(!truck.auto_steering ||
// 			truck.reverse)
// 		{
// // continue manual steering from before throttle press
//             truck.steering_pwm = truck.mid_steering_pwm +
//                 (truck.steering - truck.remote_steering_mid - truck.remote_steering_deadband) *
//                 truck.max_steering_magnitude /
//                 (truck.remote_steering_max - truck.remote_steering_mid - truck.remote_steering_deadband);
//             *steering_overshoot = 0;
// 			truck.need_steering_feedback = 0;
// 		}
// 		else
// 		{
// // step target heading
//             float step = truck.min_steering_step +
//                 (truck.steering - truck.remote_steering_mid - truck.remote_steering_deadband) *
//                 (truck.max_steering_step - truck.min_steering_step) /
//                 (truck.remote_steering_max - truck.remote_steering_mid - truck.remote_steering_deadband);
//             truck.target_heading += step;
// 			truck.target_heading = fix_angle(truck.target_heading);
// 			truck.need_steering_feedback = 1;
//         }
//     }
//     else
//     if(truck.steering < truck.remote_steering_mid - truck.remote_steering_deadband)
//     {
// // proportional left
// 		if(!truck.auto_steering ||
// 			truck.reverse)
// 		{
// // continue manual steering from before throttle press
//             truck.steering_pwm = truck.mid_steering_pwm -
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.steering) *
//                 truck.max_steering_magnitude /
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.remote_steering_min);
//             *steering_overshoot = 0;
// 			truck.need_steering_feedback = 0;
//         }
//         else
//         {
// // step target heading
//             float step = truck.min_steering_step +
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.steering) *
//                 (truck.max_steering_step - truck.min_steering_step) /
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.remote_steering_min);
//             truck.target_heading -= step;
//             truck.target_heading = fix_angle(truck.target_heading);
//             truck.need_steering_feedback = 1;
//         }
//     }
// }

void handle_steering()
{
    int throttle_active = 0;
    float steering_overshoot = 0;
    
    if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband ||
        truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
    {
        throttle_active = 1;
    }

// manual steering
#ifdef BINARY_STEERING
    if(!throttle_active || !truck.auto_steering)
    {
#endif
// steering inactive
        if(truck.steering <= truck.remote_steering_mid + truck.remote_steering_deadband &&
            truck.steering >= truck.remote_steering_mid - truck.remote_steering_deadband)
        {
		    truck.steering_pwm = truck.mid_steering_pwm;
		    truck.auto_steering = 1;
            if(throttle_active)
            {
    		    truck.need_steering_feedback = 1;
            }
        }
        else
        if(truck.steering > truck.remote_steering_mid + truck.remote_steering_deadband)
        {
// left
            if(truck.steering >= truck.remote_steering_max)
            {
                truck.steering_pwm = truck.mid_steering_pwm + truck.max_steering_magnitude;
            }
            else
            {
                truck.steering_pwm = truck.mid_steering_pwm + truck.min_steering_magnitude;
            }

//             truck.steering_pwm = truck.mid_steering_pwm +
//                 (truck.steering - truck.remote_steering_mid - truck.remote_steering_deadband) *
//                 truck.max_steering_magnitude /
//                 (truck.remote_steering_max - truck.remote_steering_mid - truck.remote_steering_deadband);
		    truck.auto_steering = 0;
		    truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
        }
        else
        if(truck.steering < truck.remote_steering_mid - truck.remote_steering_deadband)
        {
// right
            if(truck.steering <= truck.remote_steering_min)
            {
                truck.steering_pwm = truck.mid_steering_pwm - truck.max_steering_magnitude;
            }
            else
            {
                truck.steering_pwm = truck.mid_steering_pwm - truck.min_steering_magnitude;
            }

//             truck.steering_pwm = truck.mid_steering_pwm -
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.steering) *
//                 truck.max_steering_magnitude /
//                 (truck.remote_steering_mid - truck.remote_steering_deadband - truck.remote_steering_min);
		    truck.auto_steering = 0;
		    truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
        }
#ifdef BINARY_STEERING
    }
    else
// binary steering
    {
        switch(truck.binary_steering)
        {
            case FAST_LEFT:
                if(truck.reverse)
                {
                    truck.target_heading += truck.max_steering_step / TIMER_HZ;
                }
                else
                {
                    truck.target_heading -= truck.max_steering_step / TIMER_HZ;
                }
                truck.target_heading = fix_angle(truck.target_heading);
                truck.need_steering_feedback = 1;
                break;

            case SLOW_LEFT:
                if(truck.reverse)
                {
                    truck.target_heading += truck.min_steering_step / TIMER_HZ;
                }
                else
                {
                    truck.target_heading -= truck.min_steering_step / TIMER_HZ;
                }
                
                truck.target_heading = fix_angle(truck.target_heading);
                truck.need_steering_feedback = 1;
                break;

            case SLOW_RIGHT:
                if(truck.reverse)
                {
                    truck.target_heading -= truck.min_steering_step / TIMER_HZ;
                }
                else
                {
                    truck.target_heading += truck.min_steering_step / TIMER_HZ;
                }
                truck.target_heading = fix_angle(truck.target_heading);
                truck.need_steering_feedback = 1;
                break;

            case FAST_RIGHT:
                if(truck.reverse)
                {
                    truck.target_heading -= truck.max_steering_step / TIMER_HZ;
                }
                else
                {
                    truck.target_heading += truck.max_steering_step / TIMER_HZ;
                }
                truck.target_heading = fix_angle(truck.target_heading);
                truck.need_steering_feedback = 1;
                break;
            
            default:
                truck.need_steering_feedback = 1;
                break;
        }
    }
#endif // BINARY_STEERING

// auto steering
    if(truck.need_steering_feedback)
    {
        float steering_feedback100;
	    if(truck.steering_timeout > 0)
	    {
		    truck.steering_timeout--;
	    }
	    else
	    {
// disable heading hold after a certain time with no throttle or a change in reverse
		    truck.need_steering_feedback = 0;
	    }					



// disable feedback if the gyro malfunctioned
        if(!truck.imu.gyro_valid)
        {
            steering_feedback100 = 0;
        }
        else
        {
	        int raw_gyro = truck.imu.gyro_z_centered;

    // filtered heading errors
	        float error = -get_angle_change(truck.current_heading, truck.target_heading);

    // reverse feedback if going in reverse
            if(truck.reverse)
            {
                error = -error;
            }
	        float p = error;
	        float d = do_highpass(&truck.d_filter, error);
	        float d2 = (float)raw_gyro / truck.angle_to_gyro;

            truck.imu.gyro_valid = 0;
	        steering_feedback100 = do_pid(&truck.heading_pid, 
		        p, 
		        d, 
		        d2);
        }

	    truck.steering_pwm = truck.mid_steering_pwm -
		    steering_feedback100 * 
		    truck.max_steering_magnitude / 
		    100;

// static int debug_counter = 0;
// debug_counter++;
// if((debug_counter % 10) == 0)
// {
// TRACE2
// print_text("current_heading=");
// print_float(TO_DEG(truck.current_heading));
// print_text("target_heading=");
// print_float(TO_DEG(truck.target_heading));
// print_text("steering_feedback100=");
// print_float(steering_feedback100);
// print_text("steering_pwm=");
// print_number(truck.steering_pwm);
// print_lf();
//    }
    }
    else
    {
// manual steering
        reset_filter(&truck.d_filter);
	    reset_pid(&truck.heading_pid);


	    truck.target_heading = truck.current_heading +
            steering_overshoot;

        truck.target_heading = fix_angle(truck.target_heading);
    }
}




void do_auto_throttle()
{
// convert stick to RPM
    truck.target_rpm = 0;
    if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband)
    {
// forward RPM
            truck.target_rpm = truck.max_rpm;
//         if(truck.throttle >= truck.remote_throttle_max)
//         {
//             truck.target_rpm = truck.max_rpm;
//         }
//         else
//         {
//             truck.target_rpm = truck.min_rpm;
//         }

//         truck.target_rpm = truck.min_rpm +
//             (truck.throttle - truck.remote_throttle_mid - truck.remote_throttle_deadband) *
//             (truck.max_rpm - truck.min_rpm) /
//             (truck.remote_throttle_max - truck.remote_throttle_mid - truck.remote_throttle_deadband);
        truck.reverse = 0;
        CLAMP(truck.target_rpm, truck.min_rpm, truck.max_rpm);

// adjust based on speed paddles
        if(truck.speed_offset != 0)
        {
            float M_TO_MI = 1609.0;
            float PI = 3.14159;
// convert RPM to miles per minute
            float pace = truck.target_rpm * truck.diameter * PI / 1000 / M_TO_MI;
// convert RPM to minutes per mile
            pace = 1.0f / pace;
// .5 minutes per mile per speed offset
            pace += .5 * -truck.speed_offset;

            truck.target_rpm = M_TO_MI / 
                pace / 
                (PI * truck.diameter / 1000);
        }
    }
    else
    if(truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
    {
// reverse RPM
        truck.target_rpm = truck.max_reverse_rpm;
//         if(truck.throttle <= truck.remote_throttle_min)
//         {
//             truck.target_rpm = truck.max_reverse_rpm;
//         }
//         else
//         {
//             truck.target_rpm = truck.min_reverse_rpm;
//         }

//         truck.target_rpm = truck.min_reverse_rpm +
//             (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.throttle) *
//             (truck.max_reverse_rpm - truck.min_reverse_rpm) /
//             (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.remote_throttle_min);
        truck.reverse = 1;
        CLAMP(truck.target_rpm, truck.min_reverse_rpm, truck.max_rpm);
    }


    int throttle_base;
    if(!truck.reverse)
    {
        throttle_base = truck.throttle_base100;
    }
    else
    {
        throttle_base = truck.throttle_reverse_base100;
    }

// static int debug_counter = 0;
// debug_counter++;
// if(!(debug_counter % 10))
// {
// TRACE2
// print_number(truck.target_rpm);
// }

	truck.throttle_feedback = do_pid(&truck.rpm_pid,
		(float)(truck.target_rpm - truck.rpm) / 1000,
		-get_derivative(&truck.rpm_dv) / 1000,
		0);
// convert percent to PWM
    truck.power = (throttle_base + truck.throttle_feedback) * 
        MOTOR_PWM_PERIOD / 
        100;
// don't go into braking mode or it'll oscillate
    CLAMP(truck.power, 1, MOTOR_PWM_PERIOD);
}

void do_manual_throttle()
{
// go into auto throttle if fully deflected
    if(truck.throttle >= truck.remote_throttle_max)
    {
        truck.auto_throttle = 1;
        truck.reverse = 0;
    }
    else
    if(truck.throttle <= truck.remote_throttle_min)
    {
        truck.auto_throttle = 1;
        truck.reverse = 1;
    }
    else
// manual control
    if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband)
    {
// full power for 1 cycle
//        truck.power = MOTOR_PWM_PERIOD * truck.throttle_base100 / 100;
        truck.power = MOTOR_PWM_PERIOD * 
            truck.throttle_base100 *
            (truck.throttle - truck.remote_throttle_mid - truck.remote_throttle_deadband) /
            (truck.remote_throttle_max - truck.remote_throttle_mid - truck.remote_throttle_deadband) /
            100;
//        truck.auto_throttle = 1;
        truck.reverse = 0;
    }
    else
    if(truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
    {
// full power for 1 cycle
//        truck.power = MOTOR_PWM_PERIOD * truck.throttle_base100 / 100;
        truck.power = MOTOR_PWM_PERIOD * 
            truck.throttle_reverse_base100 *
            (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.throttle) /
            (truck.remote_throttle_mid - truck.remote_throttle_deadband - truck.remote_throttle_min) /
            100;
//        truck.auto_throttle = 1;
        truck.reverse = 1;
    }
}





void handle_bt_input(int throttle_magnitude)
{
// always manual steering
	truck.need_steering_feedback = 0;
// 	int target_throttle = truck.mid_throttle_pwm -
// 		throttle_magnitude * 
// 		truck.bt_throttle / 
// 		127;
// // forward
// 	if(truck.bt_throttle > 0)
// 	{
// // decrease PWM to target
// 		if(truck.throttle_pwm > target_throttle)
// 		{
// 			truck.throttle_ramp_counter++;
// 			if(truck.throttle_ramp_counter >= 
// 				truck.throttle_ramp_delay * PWM_HZ / PWM_BASE)
// 			{
// 				truck.throttle_ramp_counter = 0;
// 				truck.throttle_pwm -= truck.throttle_ramp_step;
// 				truck.throttle_pwm = MAX(truck.throttle_pwm, 
// 					truck.mid_throttle_pwm -
// 					throttle_magnitude);
// 			}
// 		}
// 		else
// 		{
// 			truck.throttle_pwm = target_throttle;
// 		}
// 	}
// 	else
// // reverse
// 	if(truck.bt_throttle < 0)
// 	{
// // increase PWM to target
// 		if(truck.throttle_pwm < target_throttle)
// 		{
// 			truck.throttle_ramp_counter++;
// 			if(truck.throttle_ramp_counter >= 
// 				truck.throttle_ramp_delay * PWM_HZ / PWM_BASE)
// 			{
// 				truck.throttle_ramp_counter = 0;
// 				truck.throttle_pwm += truck.throttle_ramp_step;
// 				truck.throttle_pwm = MIN(truck.throttle_pwm, 
// 					truck.mid_throttle_pwm +
// 					throttle_magnitude);
// 			}
// 		}
// 		else
// 		{
// 			truck.throttle_pwm = target_throttle;
// 		}
// 	}
// 	else
// 	{
// // no bluetooth throttle
// 		truck.throttle_pwm = truck.mid_throttle_pwm;
// 	}
// 

	truck.steering_pwm = truck.mid_steering_pwm +
		truck.max_steering_magnitude *
		truck.bt_steering / 
		127;
}




// PWM for steering has to be an interrupt handler since it's on the wrong pin
void feedback()
{
    if(truck.testing_motors) return;
	if(truck.writing_settings) return;

// -100 - 100
	float steering_overshoot = 0;


	if(truck.have_gyro_center)
	{
// cancel bluetooth controls
		if(truck.have_bt_controls && truck.bt_timeout <= 0)
		{
			truck.have_bt_controls = 0;
		}

// cancel radio controls
		if(truck.radio_timeout <= 0)
		{
			truck.throttle = truck.remote_throttle_mid;
			truck.steering = truck.remote_steering_mid;
		}

// cut in throttle
        if(truck.throttle >= truck.remote_throttle_mid - truck.remote_throttle_deadband &&
            truck.throttle <= truck.remote_throttle_mid + truck.remote_throttle_deadband)
        {
// back to direct voltage control
            truck.auto_throttle = 0;
        }

// Change in throttle direction
 		if((truck.throttle2 < truck.remote_throttle_mid - truck.remote_throttle_deadband && 
            truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband) ||
 			(truck.throttle2 > truck.remote_throttle_mid + truck.remote_throttle_deadband && 
            truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband))
 		{
// heading is reset after a timeout or steering command after stopping
            truck.current_heading = 0;
            truck.target_heading = 0;
            truck.steering_timeout = 0;
// go to direct voltage control
            truck.auto_throttle = 0;

	        reset_pid(&truck.heading_pid);
	        reset_pid(&truck.rpm_pid);
            reset_derivative(&truck.rpm_dv);
 		}

// replace previous throttle value if the motor moves
  		if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband ||
            truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
        {
            truck.throttle2 = truck.throttle;
        }

        
// Store the new heading if steering reversed or stopped
 		if((truck.steering2 < truck.remote_steering_mid - truck.remote_steering_deadband && 
            truck.steering >= truck.remote_steering_mid - truck.remote_steering_deadband) ||
 			(truck.steering2 > truck.remote_steering_mid + truck.remote_steering_deadband && 
            truck.steering <= truck.remote_steering_mid + truck.remote_steering_deadband))
 		{
            truck.target_heading = truck.current_heading;
	        truck.steering2 = truck.steering;
        }



 		int throttle_magnitude = MOTOR_PWM_PERIOD;
// 		if(truck.have_bt_controls && truck.bt_throttle < 0)
// 		{
// 			throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
// 				truck.max_throttle_rev100 / 
// 				100;
// // don't use steering feedback in reverse
// 			truck.need_steering_feedback = 0;
// 		}
// 		else
// 		{
// 			throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
// 				truck.max_throttle_fwd100 / 
// 				100;
// 		}

		if(truck.have_bt_controls)
		{
            handle_bt_input(throttle_magnitude);
		}
		else
// stick controller
		if(truck.throttle > truck.remote_throttle_mid + truck.remote_throttle_deadband ||
            truck.throttle < truck.remote_throttle_mid - truck.remote_throttle_deadband)
		{
			truck.steering_timeout = STEERING_RELOAD;


            if(truck.auto_throttle)
            {
                do_auto_throttle();
            }
            else
            {
                do_manual_throttle();
            }
		}
		else
		{
			truck.power = 0;

		}

        handle_steering();


// TRACE2
// print_text("throttle_state=");
// print_number(truck.throttle_state);
// print_text(" truck.mid_throttle_pwm=");
// print_float(truck.mid_throttle_pwm);
// print_text(" truck.throttle_pwm=");
// print_number(truck.throttle_pwm);
// 
// 		if(truck.need_steering_feedback)
// 		{
//             auto_steering_output();
// 
// 		}
// 		else
// 		{
//             manual_steering_output(steering_overshoot);
// 		}






        truck.throttle_accum += truck.power;
        truck.throttle_count++;


		write_pwm();
	} // have_gyro_center
	else
	{
        truck.power = 0;
		truck.steering_pwm = truck.mid_steering_pwm;

		write_pwm();
	}
}


// update PWM for servo
void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_FLAG_Update)
	{
		TIM2->SR = ~TIM_FLAG_Update;
		SET_PIN(GPIOA, GPIO_Pin_11);
    }

	if(TIM2->SR & TIM_FLAG_CC2)
	{
		TIM2->SR = ~TIM_FLAG_CC2;
		CLEAR_PIN(GPIOA, GPIO_Pin_11);
	}
}

void write_pwm()
{
// TRACE2
// print_number(truck.mid_steering_pwm);
// print_number(truck.steering_pwm);
// print_lf();
#ifdef REVERSE_STEERING
	int mid_steering_pwm = (MIN_PWM + MAX_PWM) / 2;
	truck.steering_pwm = mid_steering_pwm - 
		(truck.steering_pwm - mid_steering_pwm);
#endif



	CLAMP(truck.steering_pwm, MIN_PWM, MAX_PWM);
	SET_COMPARE(TIM2, CCR2, truck.steering_pwm);

}

void init_steering()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);




	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = SERVO_PWM_PERIOD;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = truck.steering_pwm;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    write_pwm();
  	TIM_Cmd(TIM2, ENABLE);



 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC2, ENABLE);
}



// write motor phases & power to hardware
void write_motors()
{
    int lphase = truck.motors[LEFT_MOTOR].phase;
    int rphase = truck.motors[RIGHT_MOTOR].phase;



// drive 2 phases
// phase relative to the commutations is always the same
// prev_phase < 0 is the starting condition
#define MOTOR_MACRO(phase, \
    prev_phase, \
    ccr1, \
    ccr2, \
    ccr3, \
    en1_gpio, \
    en1_pin, \
    en2_gpio, \
    en2_pin, \
    en3_gpio, \
    en3_pin) \
    if(phase >= 0 && phase < 120) \
    { \
        if(!truck.reverse && (prev_phase < 0 || prev_phase == 240) || \
            truck.reverse && (prev_phase < 0 || prev_phase == 120)) \
        { \
            prev_phase = 0; \
            ccr1 = truck.power; \
            ccr2 = 0; \
            ccr3 = 0; \
	        SET_PIN(en1_gpio, en1_pin); \
	        SET_PIN(en2_gpio, en2_pin); \
	        CLEAR_PIN(en3_gpio, en3_pin); \
        } \
/* print_text("1"); */ \
    } \
    else \
    if(phase >= 120 && phase < 240) \
    { \
        if(!truck.reverse && (prev_phase < 0 || prev_phase == 0) || \
            truck.reverse && (prev_phase < 0 || prev_phase == 240)) \
        { \
            prev_phase = 120; \
            ccr1 = 0; \
            ccr2 = truck.power; \
            ccr3 = 0; \
	        CLEAR_PIN(en1_gpio, en1_pin); \
	        SET_PIN(en2_gpio, en2_pin); \
	        SET_PIN(en3_gpio, en3_pin); \
        } \
/* print_text("2"); */ \
    } \
    else \
    if(phase >= 240) \
    { \
        if(!truck.reverse && (prev_phase < 0 || prev_phase == 120) || \
            truck.reverse && (prev_phase < 0 || prev_phase == 0)) \
        { \
            prev_phase = 240; \
            ccr1 = 0; \
            ccr2 = 0; \
            ccr3 = truck.power; \
	        SET_PIN(en1_gpio, en1_pin); \
	        CLEAR_PIN(en2_gpio, en2_pin); \
	        SET_PIN(en3_gpio, en3_pin); \
        } \
/* print_text("3"); */ \
    }


// brake 
    if(truck.power == 0)
    {
	    TIM5->CCR1 = 0;
	    TIM5->CCR3 = 0;
	    TIM5->CCR2 = 0;
	    SET_PIN(LEFT_EN1_GPIO, LEFT_EN1_PIN);
	    SET_PIN(LEFT_EN2_GPIO, LEFT_EN2_PIN);
	    SET_PIN(LEFT_EN3_GPIO, LEFT_EN3_PIN);
	    TIM3->CCR1 = 0;
	    TIM3->CCR3 = 0;
	    TIM3->CCR2 = 0;
        SET_PIN(RIGHT_EN1_GPIO, RIGHT_EN1_PIN);
        SET_PIN(RIGHT_EN2_GPIO, RIGHT_EN2_PIN);
        SET_PIN(RIGHT_EN3_GPIO, RIGHT_EN3_PIN);
        return;
    }


// left wheel
// drive all 3 phases
//    if(!truck.auto_throttle)
    if(1)
    {
	    int index1 = (lphase * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	    int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	    int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;


// reverse direction by swapping a CCR
	    TIM5->CCR1 = CALCULATE_WAVEFORM(index1);
	    TIM5->CCR2 = CALCULATE_WAVEFORM(index2);
	    TIM5->CCR3 = CALCULATE_WAVEFORM(index3);
	    SET_PIN(LEFT_EN1_GPIO, LEFT_EN1_PIN);
	    SET_PIN(LEFT_EN2_GPIO, LEFT_EN2_PIN);
	    SET_PIN(LEFT_EN3_GPIO, LEFT_EN3_PIN);
    }
    else
    {
        MOTOR_MACRO(lphase, \
            truck.motors[LEFT_MOTOR].prev_phase, \
            TIM5->CCR1, \
            TIM5->CCR2, \
            TIM5->CCR3, \
            LEFT_EN1_GPIO, \
            LEFT_EN1_PIN, \
            LEFT_EN2_GPIO, \
            LEFT_EN2_PIN, \
            LEFT_EN3_GPIO, \
            LEFT_EN3_PIN)
    }


// right wheel
//    if(!truck.auto_throttle)
    if(1)
    {
	    int index1 = (rphase * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	    int index2 = (index1 + 120 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;
	    int index3 = (index1 + 240 * WAVEFORM_SIZE / 360) % WAVEFORM_SIZE;

// reverse direction by swapping a CCR
	    TIM3->CCR1 = CALCULATE_WAVEFORM(index1);
	    TIM3->CCR3 = CALCULATE_WAVEFORM(index2);
	    TIM3->CCR2 = CALCULATE_WAVEFORM(index3);
        SET_PIN(RIGHT_EN1_GPIO, RIGHT_EN1_PIN);
        SET_PIN(RIGHT_EN2_GPIO, RIGHT_EN2_PIN);
        SET_PIN(RIGHT_EN3_GPIO, RIGHT_EN3_PIN);
    }
    else
    {
        MOTOR_MACRO(rphase, \
            truck.motors[RIGHT_MOTOR].prev_phase, \
            TIM3->CCR1, \
            TIM3->CCR2, \
            TIM3->CCR3, \
            RIGHT_EN1_GPIO, \
            RIGHT_EN1_PIN, \
            RIGHT_EN2_GPIO, \
            RIGHT_EN2_PIN, \
            RIGHT_EN3_GPIO, \
            RIGHT_EN3_PIN)
    }
}




void init_motors()
{
    int i;
    for(i = 0; i < MOTORS; i++)
    {
        truck.motors[i].prev_phase = -1;
    }

// PWM pins
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5);
 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5);
 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = 
        GPIO_Pin_0 | 
		GPIO_Pin_1 | 
		GPIO_Pin_2 | 
		GPIO_Pin_6 | 
		GPIO_Pin_7;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = CLOCKSPEED / 
        (MOTOR_PWM_PERIOD + 1) /
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

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
  	TIM_Cmd(TIM3, ENABLE);
 	TIM_CtrlPWMOutputs(TIM3, ENABLE);


    truck.power = 0;
    truck.motors[LEFT_MOTOR].phase = 0;
    truck.motors[RIGHT_MOTOR].phase = 0;


    write_motors();

// Enable pins
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_Pin = LEFT_EN1_PIN;
	GPIO_Init(LEFT_EN1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LEFT_EN2_PIN;
	GPIO_Init(LEFT_EN2_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LEFT_EN3_PIN;
	GPIO_Init(LEFT_EN3_GPIO, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = RIGHT_EN1_PIN;
	GPIO_Init(RIGHT_EN1_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = RIGHT_EN2_PIN;
	GPIO_Init(RIGHT_EN2_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = RIGHT_EN3_PIN;
	GPIO_Init(RIGHT_EN3_GPIO, &GPIO_InitStructure);

	CLEAR_PIN(LEFT_EN1_GPIO, LEFT_EN1_PIN);
	CLEAR_PIN(LEFT_EN2_GPIO, LEFT_EN2_PIN);
	CLEAR_PIN(LEFT_EN3_GPIO, LEFT_EN3_PIN);

	CLEAR_PIN(RIGHT_EN1_GPIO, RIGHT_EN1_PIN);
	CLEAR_PIN(RIGHT_EN2_GPIO, RIGHT_EN2_PIN);
	CLEAR_PIN(RIGHT_EN3_GPIO, RIGHT_EN3_PIN);
}



void init_halls()
{
    int i;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

// enable ground pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    CLEAR_PIN(GPIOB, GPIO_Pin_1);


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

// left motor
// must call this before ENABLE
	ADC_Init(ADC1, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC1, 
		ADC_Channel_4, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);

// right motor
// must call this before ENABLE
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_RegularChannelConfig(ADC2, 
		ADC_Channel_14, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC2, ENABLE);

// start the conversions
	ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	ADC2->CR2 |= (uint32_t)ADC_CR2_SWSTART;



    truck.current_hall = 0;
    
}

void handle_halls()
{
    if((ADC1->SR & ADC_FLAG_EOC) &&
        (ADC2->SR & ADC_FLAG_EOC))
    {
        if(truck.current_hall == 0)
        {
            truck.current_hall = 1;
            truck.halls[LEFT_HALL].accum += ADC1->DR;
            truck.halls[LEFT_HALL].readings++;
            truck.halls[RIGHT_HALL].accum += ADC2->DR;
            truck.halls[RIGHT_HALL].readings++;
// change channels
            ADC_RegularChannelConfig(ADC1, 
		        ADC_Channel_5, 
		        1, 
		        ADC_SampleTime_480Cycles);
            ADC_RegularChannelConfig(ADC2, 
		        ADC_Channel_15, 
		        1, 
		        ADC_SampleTime_480Cycles);
        }
        else
        {
            truck.current_hall = 0;
            truck.halls[LEFT_HALL + 1].accum += ADC1->DR;
            truck.halls[LEFT_HALL + 1].readings++;
            truck.halls[RIGHT_HALL + 1].accum += ADC2->DR;
            truck.halls[RIGHT_HALL + 1].readings++;
// change channels
            ADC_RegularChannelConfig(ADC1, 
		        ADC_Channel_4, 
		        1, 
		        ADC_SampleTime_480Cycles);
            ADC_RegularChannelConfig(ADC2, 
		        ADC_Channel_14, 
		        1, 
		        ADC_SampleTime_480Cycles);
        }
// start the conversions
	    ADC1->CR2 |= (uint32_t)ADC_CR2_SWSTART;
	    ADC2->CR2 |= (uint32_t)ADC_CR2_SWSTART;
    }
}



void handle_motors()
{
#define GET_HALLS \
    truck.halls[0].value = truck.halls[0].accum / truck.halls[0].readings; \
    truck.halls[1].value = truck.halls[1].accum / truck.halls[1].readings; \
    truck.halls[2].value = truck.halls[2].accum / truck.halls[2].readings; \
    truck.halls[3].value = truck.halls[3].accum / truck.halls[3].readings; \
    truck.halls[0].accum = 0; \
    truck.halls[0].readings = 0; \
    truck.halls[1].accum = 0; \
    truck.halls[1].readings = 0; \
    truck.halls[2].accum = 0; \
    truck.halls[2].readings = 0; \
    truck.halls[3].accum = 0; \
    truck.halls[3].readings = 0;


// handle commutations
    if(!truck.testing_motors &&
        truck.have_gyro_center &&
        truck.halls[LEFT_HALL].readings >= HALL_OVERSAMPLE &&
        truck.halls[LEFT_HALL + 1].readings >= HALL_OVERSAMPLE &&
        truck.halls[RIGHT_HALL].readings >= HALL_OVERSAMPLE &&
        truck.halls[RIGHT_HALL + 1].readings >= HALL_OVERSAMPLE)
    {
        DISABLE_INTERRUPTS
        GET_HALLS
        ENABLE_INTERRUPTS


        int left_angle = get_motor_angle(LEFT_MOTOR, LEFT_HALL);
        int right_angle = get_motor_angle(RIGHT_MOTOR, RIGHT_HALL);
        int prev_left = truck.motors[LEFT_MOTOR].angle;
        int prev_right = truck.motors[RIGHT_MOTOR].angle;
// how far ahead the magnets should pull
        int phase_offset = 90;
//         if(truck.auto_throttle)
//         {
//             phase_offset = 120;
//         }
        
        truck.motors[LEFT_MOTOR].angle = left_angle;
        truck.motors[RIGHT_MOTOR].angle = right_angle;


// tachometer
        if(!truck.reverse)
        {
// wrapped around
            if(prev_left > 240 && left_angle < 90)
            {
                prev_left -= 360;
            }
            else
// went backward (stall)
            if(prev_left < 90 && left_angle > 240)
            {
                left_angle -= 360;
            }


// wrapped around
            if(prev_right > 240 && right_angle < 90)
            {
                prev_right -= 360;
            }
            else
// went backward (stall)
            if(prev_right < 90 && right_angle > 240)
            {
                right_angle -= 360;
            }

            int l_diff = left_angle - prev_left;
            int r_diff = right_angle - prev_right;
            truck.motors[LEFT_MOTOR].v_temp += l_diff;
            truck.motors[RIGHT_MOTOR].v_temp += r_diff;

// advance phase
            if(l_diff >= 0)
            {
                truck.motors[LEFT_MOTOR].phase = ((left_angle * 7) + phase_offset) % 360;
            }

            if(r_diff >= 0)
            {
                truck.motors[RIGHT_MOTOR].phase = ((right_angle * 7) + phase_offset) % 360;
            }
        }
        else
// reverse
        {
// wrapped around
            if(prev_left < 90 && left_angle > 240)
            {
                prev_left += 360;
            }
            else
// went backward (stall)
            if(prev_left > 240 && left_angle < 90)
            {
                left_angle += 360;
            }

// wrapped around
            if(prev_right < 90 && right_angle > 240)
            {
                prev_right += 360;
            }
            else
// went backward (stall)
            if(prev_right > 240 && right_angle < 90)
            {
                right_angle += 360;
            }

            int l_diff = left_angle - prev_left;
            int r_diff = right_angle - prev_right;
            truck.motors[LEFT_MOTOR].v_temp -= l_diff;
            truck.motors[RIGHT_MOTOR].v_temp -= r_diff;
            
// advance phase
            if(l_diff <= 0)
            {
                truck.motors[LEFT_MOTOR].phase = (left_angle * 7) - phase_offset;
                if(truck.motors[LEFT_MOTOR].phase < 0)
                {
                    truck.motors[LEFT_MOTOR].phase += 360;
                }
                truck.motors[LEFT_MOTOR].phase %= 360;
            }

            if(r_diff <= 0)
            {
                truck.motors[RIGHT_MOTOR].phase = (right_angle * 7) - phase_offset;
                if(truck.motors[RIGHT_MOTOR].phase < 0)
                {
                    truck.motors[RIGHT_MOTOR].phase += 360;
                }
                truck.motors[RIGHT_MOTOR].phase %= 360;
            }   
        }




        write_motors();

    }

    if(truck.tick >= truck.rpm_tick)
    {
        truck.rpm_tick = truck.tick + TIMER_HZ / RPM_HZ;

        truck.motors[LEFT_MOTOR].v = truck.motors[LEFT_MOTOR].v_temp;
        truck.motors[LEFT_MOTOR].v_temp = 0;
        truck.motors[RIGHT_MOTOR].v = truck.motors[RIGHT_MOTOR].v_temp;
        truck.motors[RIGHT_MOTOR].v_temp = 0;
// average L & R angular velocity
        truck.rpm = (truck.motors[LEFT_MOTOR].v + truck.motors[RIGHT_MOTOR].v) *
            10 *
            60 / 
            360 / 
            2;
        if(truck.rpm < 0)
        {
            truck.rpm = 0;
        }

        update_derivative(&truck.rpm_dv, truck.rpm);

// angular rate & mph
//TRACE2
// print_float(truck.motors[LEFT_MOTOR].v);
// print_float(truck.motors[RIGHT_MOTOR].v);
// print_float(V_TO_MPH(truck.motors[LEFT_MOTOR].v));
// print_float(V_TO_MPH(truck.motors[RIGHT_MOTOR].v));
//print_number(truck.rpm);
//print_number(truck.target_rpm);
//print_number(truck.power);
//print_number(truck.motors[LEFT_MOTOR].angle);
//print_number(truck.motors[LEFT_MOTOR].phase);
    }

// print debug
    if(truck.debug_tick <= truck.tick &&
        truck.have_gyro_center)
    {
        truck.debug_tick = truck.tick + TIMER_HZ / 10;



//TRACE2
//print_number(debug_rpm * 6);
//debug_rpm = 0;
//print_number(truck.halls[LEFT_MOTOR].value);
//print_number(truck.halls[LEFT_MOTOR + 1].value);
//print_number(truck.motors[LEFT_MOTOR].angle);
//print_number(truck.motors[LEFT_MOTOR].phase);
//print_lf();
    }




// motor calibration
    if(truck.testing_motors)
    {
// get motor positions
        if(truck.tick >= truck.test_tick &&
            truck.halls[LEFT_HALL].readings >= HALL_OVERSAMPLE &&
            truck.halls[LEFT_HALL + 1].readings >= HALL_OVERSAMPLE &&
            truck.halls[RIGHT_HALL].readings >= HALL_OVERSAMPLE &&
            truck.halls[RIGHT_HALL + 1].readings >= HALL_OVERSAMPLE)
        {
            GET_HALLS
            switch(truck.test_state)
            {
                case START_TEST:
                    truck.test_phase = 0;
                    truck.motors[LEFT_MOTOR].phase = 0;
                    truck.motors[RIGHT_MOTOR].phase = 0;
                    write_motors();
                    truck.test_tick = truck.tick + TIMER_HZ;
                    truck.test_state = TEST_PASS1;
                    break;

// 1 revolution for the starting points
                case TEST_PASS1:
                    truck.test_tick = truck.tick + TIMER_HZ / 10;

                    truck.test_phase += ANGLE_STEP;
                    if(truck.test_phase >= 360 * 7)
                    {
                        truck.test_state = TEST_PASS2;
                        truck.test_phase = 0;
                        truck.test_counter = 0;
                        truck.motors[LEFT_MOTOR].phase = 0;
                        truck.motors[RIGHT_MOTOR].phase = 0;
                        write_motors();
                        truck.test_tick = truck.tick + TIMER_HZ;
                    }
                    else
                    {
                        truck.motors[LEFT_MOTOR].phase = truck.test_phase % 360;
                        truck.motors[RIGHT_MOTOR].phase = truck.test_phase % 360;
                        write_motors();
                    }
                    break;


    // capture waveforms
                case TEST_PASS2:
                    truck.test_tick = truck.tick + TIMER_HZ;
                    if(truck.test_counter == 1)
                    {
                        truck.test_counter = 0;

                        if(truck.power > 0)
                        {
    // print reading for last motor phase
                            TRACE2
                            print_text("MOTORS: ");
                            print_number(truck.test_phase);
                            print_number(truck.halls[LEFT_HALL].value);
                            print_number(truck.halls[LEFT_HALL + 1].value);
                            print_number(truck.halls[RIGHT_HALL].value);
                            print_number(truck.halls[RIGHT_HALL + 1].value);
                            print_lf();
                        }

                        truck.test_phase += ANGLE_STEP;
                        if(truck.test_phase >= 360 * 7)
                        {
            // done testing
                            TRACE2
                            print_text("Test done\n");
                            truck.power = 0;
                            write_motors();
                            truck.test_state = TEST_DONE;
                            truck.testing_motors = 0;
                        }
                        else
                        {
            // advance motor phase
                            truck.motors[LEFT_MOTOR].phase = truck.test_phase % 360;
                            truck.motors[RIGHT_MOTOR].phase = truck.test_phase % 360;
                            write_motors();
                        }
                    }
                    else
                    {
                        truck.test_counter = 1;
            // begin reading halls
                    }
                    break;
            }


        }
    }
}


void init_bluetooth()
{
#ifdef BLUETOOTH_PASSTHROUGH
	truck.bluetooth.current_function = bluetooth_passthrough;
	TRACE2
	print_text("Entering bluetooth passthrough.\n");
#else
	truck.bluetooth.current_function = bt_get_code1;
#endif

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    const int UART_RX_PIN = 7;
    const int UART_TX_PIN = 6;
	GPIO_PinAFConfig(GPIOB, UART_RX_PIN, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, UART_TX_PIN, GPIO_AF_USART1);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = (1 << UART_RX_PIN) | (1 << UART_TX_PIN);
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
// once a device is configured
#ifndef BLUETOOTH9600
	USART_InitStructure.USART_BaudRate = 115200;
#else
// set to configure a new device
	USART_InitStructure.USART_BaudRate = 9600;
#endif

    TRACE2
    print_text("bluetooth baud=");
    print_number(USART_InitStructure.USART_BaudRate);
    print_lf();

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* USART configuration */
  	USART_Init(USART1, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART1, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}


void USART1_IRQHandler()
{
// got data from bluetooth
    if(USART1->SR & USART_FLAG_RXNE)
    {
    	truck.bluetooth.data = USART1->DR;
	    truck.bluetooth.current_function();
    }

}

void radio_get_code1();

void init_radio()
{
    truck.radio.current_function = radio_get_code1;

	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
    const int UART_RX_PIN = 2;
	GPIO_PinAFConfig(GPIOD, UART_RX_PIN, GPIO_AF_UART5);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = (1 << UART_RX_PIN);
  	GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* USART configuration */
  	USART_Init(UART5, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(UART5, ENABLE);
    

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

    
}

void radio_get_data()
{
    truck.radio.packet[truck.radio.counter++] = truck.radio.data;
    if(truck.radio.counter >= 3)
    {
        truck.radio.current_function = radio_get_code1;

// DEBUG
// TRACE2
// print_number(truck.radio.packet[0]);
// print_number(truck.radio.packet[1]);
// print_number(truck.radio.packet[2]);

        truck.radio.got_packet = 1;
    }
}

void radio_get_debug()
{
    if(truck.radio.data == 0)
    {
        truck.radio.current_function = radio_get_code1;
    }
    else
    {
        send_uart(&truck.radio.data, 1);
    }
}

void radio_get_code2()
{
    if(truck.radio.data == 0xff)
    {
        truck.radio.current_function = radio_get_code2;
    }
    else
    if(truck.radio.data == 0xe7)
    {
        truck.radio.current_function = radio_get_data;
        truck.radio.counter = 0;
    }
    else
    if(truck.radio.data == 0x7e)
    {
        truck.radio.current_function = radio_get_debug;
        truck.radio.counter = 0;
    }
    else
    {
        truck.radio.current_function = radio_get_code1;
    }
}

void radio_get_code1()
{
    if(truck.radio.data == 0xff)
    {
        truck.radio.current_function = radio_get_code2;
    }
    else
// pass through debug text
    {
//        send_uart(&truck.radio.data, 1);
    }
}

void UART5_IRQHandler()
{
// got data from radio
    if(UART5->SR & USART_FLAG_RXNE)
    {
    	truck.radio.data = UART5->DR;
	    truck.radio.current_function();
    }

}


#define IMU_ADDRESS (0x68 << 1)
static void imu_status1(void *ptr);
static void imu_test1(void *ptr);


static void imu_read_gyros(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
	int gyro_z = (int16_t)((imu->i2c.burst[0] << 8) | (imu->i2c.burst[1]));
    imu->total_gyro++;


#ifdef DEBUG_I2C
    capture_i2c = 0;
    int i;
    for(i = 0; i < i2c_offset; i++)
    {
        print_number((i2c_capture[i] & 2) ? 1 : 0);
        print_number(i2c_capture[i] & 1);
        print_lf();
        flush_uart();
    }
#endif

//TRACE2
//print_number(gyro_z);
//print_number(imu->i2c.value);
//print_number(PIN_IS_SET(((GPIO_TypeDef*)imu->i2c.clock_gpio), imu->i2c.clock_pin));
//print_number(PIN_IS_SET(((GPIO_TypeDef*)imu->i2c.data_gpio), imu->i2c.data_pin));


    if(!truck.have_gyro_center)
    {
        truck.led_counter++;
        if(truck.led_counter >= LED_DELAY)
        {
            TOGGLE_PIN(LED_GPIO, RED_LED);
	        truck.led_counter = 0;
        }

        imu->current_heading = imu->abs_heading;
		imu->gyro_z_accum += gyro_z;
		imu->gyro_z_min = MIN(gyro_z, imu->gyro_z_min);
		imu->gyro_z_max = MAX(gyro_z, imu->gyro_z_max);
		imu->gyro_center_count++;
        if(imu->gyro_z_max - imu->gyro_z_min > truck.gyro_center_max)
        {
			TRACE2
			print_text("range too big last_value=");
            print_number(gyro_z);
            print_text("range=");
            print_number(imu->gyro_z_max - imu->gyro_z_min);
            imu->gyro_center_count = 0;
            imu->gyro_z_accum = 0;
            imu->gyro_z_min = 65535;
            imu->gyro_z_max = -65535;
        }
        
        if(imu->gyro_center_count >= GYRO_CENTER_TOTAL)
        {
            imu->prev_gyro_z_center = imu->gyro_z_center;
            imu->gyro_z_center = (float)imu->gyro_z_accum / imu->gyro_center_count;
// test if calculation didn't drift
            if(fabs(imu->prev_gyro_z_center - imu->gyro_z_center) < 
                (float)truck.max_gyro_drift)
            {
				TRACE2
				print_text("got center center=");
                print_float(imu->gyro_z_center);
				truck.have_gyro_center = 1;
// solid red
    			SET_PIN(LED_GPIO, RED_LED);
            }
			else
// try again
			{
                TRACE2
                print_text("drift too big change=");
                print_float(imu->gyro_z_center - imu->prev_gyro_z_center);

				imu->gyro_center_count = 0;
				imu->gyro_z_accum = 0;
				imu->gyro_z_min = 65535;
				imu->gyro_z_max = -65535;
			}
        }
    }
    else
    {
// lowpass filter it
        imu->gyro_z = imu->gyro_z * 
            (1.0f - imu->gyro_bandwidth) + (float)gyro_z * imu->gyro_bandwidth;
// center it
        imu->gyro_z_centered = imu->gyro_z - imu->gyro_z_center;
        imu->gyro_valid = 1;
        
		DISABLE_INTERRUPTS
// accumulate heading
		truck.current_heading += imu->gyro_z_centered / truck.angle_to_gyro / NAV_HZ;
		truck.current_heading = fix_angle(truck.current_heading);
		ENABLE_INTERRUPTS
    }


// start gyro status read
    imu_status1(imu);
}


static int debug_counter = 0;
static void imu_status2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;
//debug_counter++;
//TRACE2
//print_number(imu->i2c.value);
	if((imu->i2c.value & 0x01))
	{
#ifdef DEBUG_I2C
        capture_i2c = 1;
        i2c_offset = 0;
#endif
//TRACE2
//print_number(debug_counter);
//debug_counter = 0;
 		i2c_read_burst(&imu->i2c,
			IMU_ADDRESS, 
 			0x47,
 			2);
//		i2c_read_device(&imu->i2c,
//			IMU_ADDRESS, 
//			0x48);
		imu->current_function = imu_read_gyros;
	}
 	else
	{
		imu_status1(ptr);
	}
}

static void imu_status1(void *ptr)
{
// get gyro status
	imu_t *imu = (imu_t*)ptr;
	i2c_read_device(&imu->i2c, IMU_ADDRESS, 0x3a);
	imu->current_function = imu_status2;
}


static void imu_config3(void *ptr)
{
// gyro full scale range
// 500 deg/sec
	imu_t *imu = (imu_t*)ptr;
	i2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1b, 0x08);
	imu->current_function = imu_status1;
	
}


static void imu_config2(void *ptr)
{
// digital low pass filter
	imu_t *imu = (imu_t*)ptr;
	i2c_write_device(&imu->i2c, IMU_ADDRESS, 0x1a, 0x0);
	imu->current_function = imu_config3;
}


static void imu_config1(void *ptr)
{
// sample rate divider
// high enough to keep i2c from dropping samples
	imu_t *imu = (imu_t*)ptr;
	i2c_write_device(&imu->i2c, IMU_ADDRESS, 0x19, 18);
//	i2c_write_device(&imu->i2c, IMU_ADDRESS, 0x19, 255);
	imu->current_function = imu_config2;
	
}

static void imu_config0(void *ptr)
{
// wake up.  Use gyro as clock source.
	imu_t *imu = (imu_t*)ptr;
	i2c_write_device(&imu->i2c, IMU_ADDRESS, 0x6b, 0x1);
	imu->current_function = imu_config1;
}

// Needs a few read requests to warm up, for some reason
static void imu_test2(void *ptr)
{
	imu_t *imu = (imu_t*)ptr;

//     TRACE2
//     print_text("device id=");
//     print_number(imu->i2c.value);

	if(imu->i2c.value == 0x68)
	{
		imu_config0(ptr);
	}
 	else
	{
		imu_test1(ptr);
	}
}

static void imu_test1(void *ptr)
{
// get gyro status
	imu_t *imu = (imu_t*)ptr;
// device ID
	i2c_read_device(&imu->i2c, IMU_ADDRESS, 0x75);
	imu->current_function = imu_test2;
}


void init_imu(imu_t *imu)
{
#ifdef ARM_HARDI2C_H
	init_hardi2c(&imu->i2c, I2C1, 1);
#endif

#ifdef ARM_SOFTI2C_H
    init_softi2c(&imu->i2c,
	    256, // delay
	    GPIOB, // data_gpio
	    GPIOB, // clock_gpio
	    GPIO_Pin_9, // data_pin
	    GPIO_Pin_8); // clock_pin
#endif

	truck.have_gyro_center = 0;

	imu->gyro_bandwidth = 1.0f;

	imu->gyro_z_min = 65535;
	imu->gyro_z_max = -65535;
	imu->current_function = imu_test1;
}

void reset_imu(imu_t *imu)
{
    imu->i2c.error = 0;
    imu->current_function = imu_test1;
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


// print the menu
void print_menu()
{
    print_text("Truck menu:\n");
    print_text("1 - test motors\n");
}

// handle user input
void handle_input()
{
    if(uart.input == '1')
    {
        TRACE2
        print_text("Starting motor test\n");
        truck.testing_motors = 1;
        truck.power = MOTOR_PWM_PERIOD / 4;
        truck.test_tick = truck.tick + TIMER_HZ;
        truck.test_state = START_TEST;
    }
    else
    if(uart.input == '\n')
    {
        print_menu();
    }
}


int main(void)
{
	int i, j;


	init_linux();
	bzero(&truck, sizeof(truck_t));


/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
			RCC_AHB1Periph_GPIOC |
			RCC_AHB1Periph_GPIOD |
			RCC_AHB1Periph_GPIOE |
			RCC_AHB1Periph_CCMDATARAMEN, 
		ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();
	


	truck.steering = 0;


	truck.mid_steering100 = 50;
	truck.max_steering100 = 100;
    truck.min_steering100 = 25;

    
    truck.remote_steering_mid = 125;
    truck.remote_steering_deadband = 2;
    truck.remote_steering_max = 255;
    truck.remote_steering_min = 3;
    
    truck.remote_throttle_mid = 124;
    truck.remote_throttle_deadband = 5;
    truck.remote_throttle_max = 254;
    truck.remote_throttle_min = 4;





	truck.gyro_center_max = 100;
	truck.max_gyro_drift = 0.1f;
	truck.angle_to_gyro = 450;
	

	
	truck.max_rpm = 500;
    truck.min_rpm = 60;
    truck.max_reverse_rpm = 300;
    truck.min_reverse_rpm = 60;

	truck.pid_downsample = 1;
	truck.min_steering_step = TO_RAD(30);
    truck.max_steering_step = TO_RAD(90);
	truck.steering_overshoot = 0;

	truck.battery_analog = 965;
	truck.battery_v0 = 8.67f;

// heading error -> PWM

	init_pid(&truck.heading_pid, 0, 0, 0, 0, 0, 0, 0);
	init_filter(&truck.p_filter, 0, 1);
	init_filter(&truck.d_filter, 0, 0);


// RPM error -> throttle
	init_pid(&truck.rpm_pid, 0, 0, 0, 0, 0, 0, 0);

	truck.rpm_dv_size = 10;
	init_derivative(&truck.rpm_dv, truck.rpm_dv_size);


// the real settings are loaded here
	load_config();
	dump_config();


    truck.steering_pwm = truck.mid_steering_pwm;


// general purpose timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = 65535;
	TIM_TimeBaseStructure.TIM_Prescaler = 25;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM10, &TIM_TimeBaseStructure);
	TIM_Cmd(TIM10, ENABLE);



// debug UART
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

	print_text("Welcome to heroine truck\n");
	flush_uart();
    init_watchdog();



// LEDs
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_Pin = RED_LED;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	SET_PIN(LED_GPIO, RED_LED);

	GPIO_InitStructure.GPIO_Pin = GREEN_LED;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	CLEAR_PIN(LED_GPIO, GREEN_LED);

// debug pin
//	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
//	GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);
//	CLEAR_PIN(DEBUG_GPIO, DEBUG_PIN);


	truck.rpm_tick = truck.tick;




	init_bluetooth();
    init_radio();
	init_analog();
	init_steering();
    init_halls();
    init_motors();

	init_imu(&truck.imu);

    print_menu();




/*
 * 	TRACE2
 * 	print_text("\nimu.angle_to_gyro=");
 * 	print_number(truck.imu.angle_to_gyro);
 */

// // DEBUG
// while(1)
// {
// 	if((USART6->SR & USART_FLAG_TC) != 0 &&
// 		uart.uart_offset < uart.uart_size)
// 	{
// 		USART6->DR = uart.uart_buffer[uart.uart_offset++];
// 	}
//     handle_i2c(&truck.imu.i2c);
// 	if(i2c_ready(&truck.imu.i2c))
// 	{
// 		truck.imu.current_function(&truck.imu);
// 	}
//     PET_WATCHDOG
// }


    int debug_phase = 0;
    int debug_rpm = 0;
	while(1)
	{
	    if((USART6->SR & USART_FLAG_TC) != 0 &&
		    uart.uart_offset < uart.uart_size)
	    {
		    USART6->DR = uart.uart_buffer[uart.uart_offset++];
	    }

		handle_bluetooth();
		handle_analog();
        handle_halls();
        handle_motors();

#ifdef ARM_HARDI2C_H
	    handle_i2c(&truck.imu.i2c);
#endif

#ifdef ARM_SOFTI2C_H
        handle_i2c_nodelay(&truck.imu.i2c);
#endif

#ifdef DEBUG_I2C
        if(capture_i2c && i2c_offset < I2C_SIZE 
    #ifdef ARM_SOFTI2C_H
        && truck.imu.i2c.delay_counter == 0
    #endif
        )
        {
            i2c_capture[i2c_offset++] = 
                (PIN_IS_SET(GPIOB, GPIO_Pin_9) << 1) | // data_pin
                (PIN_IS_SET(GPIOB, GPIO_Pin_8)); // clock_pin
        }
#endif

	    if(i2c_ready(&truck.imu.i2c))
	    {
		    truck.imu.current_function(&truck.imu);
	    }

        if(truck.imu.i2c.error)
        {
            reset_imu(&truck.imu);
// #ifdef ARM_SOFTI2C_H
// // DEBUG
// flush_uart();
// while(1) {}
// #endif
        }

        if(uart.got_input)
        {
            uart.got_input = 0;
            handle_input();
        }

		if(truck.radio.got_packet)
		{
			truck.radio.got_packet = 0;
			handle_radio_packet(truck.radio.packet);
		}

        if(truck.tick != truck.feedback_tick)
        {
            truck.feedback_tick = truck.tick;
            
            feedback();
        }


        PET_WATCHDOG
	}
}





