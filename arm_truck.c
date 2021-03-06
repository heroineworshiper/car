/*
 * STM32 Controller for converted lunchbox
 * Copyright (C) 2012-2018 Adam Williams <broadcast at earthling dot net>
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

// make truck.bin;uart_programmer truck.bin

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

// Some kind of delay in the terminal program is required to paste text in,
// but the entire command must be sent in under a second, so adjust the terminal
// program's delay.



#include "arm_truck.h"
#ifdef USE_CC1101
#include "arm_cc1101.h"
#endif

#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_flash.h"
#include "stm32f4xx_exti.h"
#include "stm32f4xx_syscfg.h"

#ifdef USE_XBEE
#include "arm_xbee.h"
#endif


#define SYNC_CODE 0xe5


// start of persistent settings
#define SETTINGS_START 0x0800c000
// end of settings
#define SETTINGS_END 0x08010000
// size of settings block
#define SETTINGS_SIZE 256
// Magic number for settings
#define SETTINGS_MAGIC 0x10291976

#define DEBUG_PIN GPIO_Pin_4
#define DEBUG_GPIO GPIOB

#ifdef USE_BRUSHLESS
// tachometer
	#define RPM_PIN GPIO_Pin_7
	#define RPM_GPIO GPIOA
#else // USE_BRUSHLESS
	#define RPM_PIN GPIO_Pin_0
	#define RPM_GPIO GPIOB
#endif // !USE_BRUSHLESS


#define SPI_GPIO GPIOB
#define MOSI_PIN GPIO_Pin_3
#define CLK_PIN GPIO_Pin_4

//#define HEADLIGHT_PIN GPIO_Pin_13
//#define HEADLIGHT_GPIO GPIOC

// packets per second
#define PACKET_RATE 40

// commands we get from the phone
#define GET_STATUS 0
#define RESET_COMMAND 1
#define NEW_CONFIG 2

// packets we send to the phone
#define STATUS_PACKET 0

#define THROTTLE_MAX 0x1
// analog range
#define STEERING_MID 0x0


#define BATTERY_OVERSAMPLE 10000
#define GYRO_OVERSAMPLE 64
#define GYRO_CENTER_TOTAL (NAV_HZ * 5)
#define CURRENT_OVERSAMPLE 5000

#define TIMEOUT_RELOAD (TIMER_HZ * 1)
#define STEERING_RELOAD (10 * PWM_HZ)

// timer for LED flashing
#define LED_DELAY (NAV_HZ / 2)
// ADC for 0 current
#define CURRENT_BASE 120.0f
// scale factor for current
#define CURRENT_SCALE 630.0f

// 50Hz
//#define PWM_PERIOD 1680000
// 100Hz
//#define PWM_PERIOD 840000
// 150Hz
//#define PWM_PERIOD 560000
// 200Hz
//#define PWM_PERIOD 420000

#define PWM_PERIOD (84000000 / PWM_HZ)

// 2ms
#define MAX_PWM 168000
// 1ms
#define MIN_PWM 84000

#define MAX_PERIOD 65535



#define RPM_FEEDBACK


truck_t truck;

typedef struct
{
	float v;
	int power;
	int rpm;
} throttle_entry_t;

const throttle_entry_t throttle_table[] = 
{
};

void write_pwm();

void imu_led_flash()
{
	truck.led_counter++;
	if(truck.led_counter >= LED_DELAY)
	{
		TOGGLE_PIN(LED_GPIO2, LED_PIN2);
		CLEAR_PIN(LED_GPIO1, LED_PIN1);
		truck.led_counter = 0;
	}
}


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
//	pid->stiction_sign = 0;
	pid->result = 0;
}

void dump_pid(pid_t *pid)
{
	print_float(pid->p_gain);
	print_float(pid->i_gain);
	print_float(pid->d_gain);
	print_float(pid->d2_gain);
//	print_float(pid->p_limit);
//	print_float(pid->i_limit);
//	print_float(pid->d_limit);
	print_lf();
}

int read_pid(pid_t *pid, const unsigned char *buffer, int offset)
{
	float p = READ_FLOAT32(buffer, offset);
	float i = READ_FLOAT32(buffer, offset);
	float d = READ_FLOAT32(buffer, offset);
	float d2 = READ_FLOAT32(buffer, offset);
// 	float p_limit = READ_FLOAT32(buffer, offset);
// 	float i_limit = READ_FLOAT32(buffer, offset);
// 	float d_limit = READ_FLOAT32(buffer, offset);
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

// fixed I factor
/*
 * 		if(pid->error_accum > 0)
 * 		{
 * 			pid->accum += pid->i_gain / NAV_HZ;
 * 		}
 * 		else
 * 		{
 * 			pid->accum -= pid->i_gain / NAV_HZ;
 * 		}
 * 	
 */

		
		CLAMP(pid->i_result, -pid->i_limit, pid->i_limit);
		pid->counter = 0;
		pid->error_accum = 0;
	}
	

	pid->result = pid->p_result + 
		pid->i_result + 
		pid->d_result + 
		pid->d2_result;




	
// 	if(pid->stiction_amount > 0.0001)
// 	{
// 		if(pid->stiction_sign > 0)
// 		{
// 			if(pid->result > pid->max_result)
// 			{
// 				pid->max_result = pid->result;
// 			}
// 			else
// // kick I in the opposite direction
// 			if(pid->result < pid->max_result - pid->stiction_threshold)
// 			{
// 				pid->stiction_sign = -1;
// 				pid->max_result = pid->result;
// 
// // I has the same sign as the error
// 				pid->accum -= pid->stiction_amount;
// 			}
// 		}
// 		else
// 		if(pid->stiction_sign < 0)
// 		{
// 			if(pid->result < pid->max_result)
// 			{
// 				pid->max_result = pid->result;
// 			}
// 			else
// // kick I in the opposite direction
// 			if(pid->result > pid->max_result + pid->stiction_threshold)
// 			{
// 				pid->stiction_sign = 1;
// // I has the same sign as the error
// 				pid->accum += pid->stiction_amount;
// 			}
// 		}
// 		else
// 		{
// // get the starting point of the stiction test
// 			if(pid->result > 0)
// 			{
// 				pid->stiction_sign = 1;
// 			}
// 			else
// 			if(pid->result < 0)
// 			{
// 				pid->stiction_sign = -1;
// 			}
// 			pid->max_result = pid->result;
// 		}
// 	}


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

#ifndef I2C_IMU
void begin_gyro_calibration()
{
TRACE2
print_text("Begin gyro calibration\n");
	DISABLE_INTERRUPTS
	truck.need_gyro_center = 1;
	truck.gyro_center_count = 0;
	truck.gyro_center = 0;
	truck.prev_gyro_center = 0;
	truck.gyro_accum = 0;
	truck.gyro_min = 0;
	truck.gyro_max = 0;
	truck.current_heading = 0;
	ENABLE_INTERRUPTS
}
#endif // !I2C_IMU


void radio_led_toggle()
{
	if(!truck.need_gyro_center)
	{
		truck.led_counter++;
		if(truck.led_counter >= LED_DELAY2)
		{
			if(truck.have_gyro_center)
			{
// flash green
				TOGGLE_PIN(LED_GPIO1, LED_PIN1);
				CLEAR_PIN(LED_GPIO2, LED_PIN2);
			}
			else
			{
// flash red
				TOGGLE_PIN(LED_GPIO2, LED_PIN2);
				CLEAR_PIN(LED_GPIO1, LED_PIN1);
			}
			truck.led_counter = 0;
		}
	}
}

void handle_radio_packet(unsigned char *ptr)
{
    truck.stick_packets++;

// packet good
	radio_led_toggle();

//TRACE2
//print_hex2(*ptr);
	DISABLE_INTERRUPTS

	truck.radio_timeout = TIMEOUT_RELOAD;
	truck.throttle_reverse = BIT_IS_CLEAR(*ptr, 0) ? 0 : 1;
	truck.throttle = BIT_IS_CLEAR(*ptr, 1) ? THROTTLE_MAX : 0;
	truck.steering = STEERING_MID;

// low speed steering
	if(BIT_IS_CLEAR(*ptr, 3)) truck.steering = 2;
	else
	if(BIT_IS_CLEAR(*ptr, 4)) truck.steering = 3;
	else
// high speed steering
	if(BIT_IS_CLEAR(*ptr, 2)) truck.steering = 1;
	else
	if(BIT_IS_CLEAR(*ptr, 5)) truck.steering = 4;

	ENABLE_INTERRUPTS


/*
* TRACE2
* print_text("reverse=");
* print_number(truck.throttle_reverse);
* print_text("throttle=");
* print_number(truck.throttle);
* print_text("steering=");
* print_number(truck.steering);
*/



// begin gyro calibration		
	if(truck.throttle >= THROTTLE_MAX && 
		!truck.have_gyro_center && 
		!truck.need_gyro_center)
	{
#ifndef I2C_IMU
		begin_gyro_calibration();
#else
		calibrate_imu(&truck.imu);
#endif

	}
//TRACE2
//print_text("mode=");
//print_number(truck.mode);
//print_text("period=");
//print_number(truck.period);
//print_text("power=");
//print_number_nospace(left_motor->power);
//print_text(",");
//print_number(right_motor->power);
}

void handle_radio()
{

#ifdef USE_HOPPING
// run the initialization functions
    if(!radio.initialized)
    {
        radio.current_function();
        return;
    }
    
// now need to know if it got a valid packet in the interrupt handler
    
    
    
#else // USE_HOPPING




	if(radio.packet[0] != SYNC_CODE) return;

	uint16_t chksum = get_chksum(radio.packet, PACKET_SIZE - 2);

	if((chksum & 0xff) == radio.packet[PACKET_SIZE - 2] &&
		((chksum >> 8) & 0xff) == radio.packet[PACKET_SIZE - 1])
	{
		handle_radio_packet(radio.packet + 2);
//		TRACE
	}
    else
    {
//        TRACE
//        print_text("bad packet");
    }

#endif // !USE_HOPPING
}

void dump_config()
{
	TRACE2
	print_text("\nmid_steering100=");
	print_number(truck.mid_steering100);
	print_text("\nmid_throttle100=");
	print_number(truck.mid_throttle100);
	print_text("\nmax_steering100=");
	print_number(truck.max_steering100);
	print_text("\nmin_steering100=");
	print_number(truck.min_steering100);
	print_text("\nauto_steering=");
	print_number(truck.auto_steering);
	print_text("\nauto_throttle=");
	print_number(truck.auto_throttle);
	print_text("\nrpm_dv_size=");
	print_float(truck.rpm_dv_size);
// 	print_text("\npath_dx_size=");
// 	print_float(truck.path_dx_size);
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


	print_text("\nmin_throttle_fwd100=");
	print_number(truck.min_throttle_fwd100);
	print_text("\nmin_throttle_rev100=");
	print_number(truck.min_throttle_rev100);

	print_text("\nmax_throttle_fwd100=");
	print_number(truck.max_throttle_fwd100);
	print_text("\nmax_throttle_rev100=");
	print_number(truck.max_throttle_rev100);

	print_text("\nthrottle_base=");
	print_number(truck.throttle_base);
	print_text("\nthrottle_reverse_base=");
	print_number(truck.throttle_reverse_base);

	print_text("\nthrottle_ramp_delay=");
	print_number(truck.throttle_ramp_delay);
	print_text("\nthrottle_ramp_step100=");
	print_number(truck.throttle_ramp_step100);



	print_text("\npid_downsample=");
	print_number(truck.pid_downsample);
	print_text("\nsteering_step_delay=");
	print_number(truck.steering_step_delay);
	print_text("\nbattery_analog=");
	print_number(truck.battery_analog);
	print_text("\ntarget_rpm=");
	print_float(truck.target_rpm);
	print_text("\ntarget_reverse_rpm=");
	print_float(truck.target_reverse_rpm);

	print_text("\nenable_mag=");
	print_number(truck.enable_mag);
	if(truck.enable_mag)
	{
		print_text("\nMAG X: ");
		print_number(truck.imu.mag_x_min);
		print_number(truck.imu.mag_x_max);
		print_text("\nMAG Y: ");
		print_number(truck.imu.mag_y_min);
		print_number(truck.imu.mag_y_max);
		print_text("\nMAG Z: ");
		print_number(truck.imu.mag_z_min);
		print_number(truck.imu.mag_z_max);
	}
	
	
//	print_text("\nenable_vision=");
//	print_number(truck.enable_vision);
//	print_text("\nvanish_center=");
//	print_number(truck.vanish_center);
//	print_text("\nbottom_center=");
//	print_number(truck.bottom_center);
//	print_text("\nvision_bandwidth=");
//	print_float(truck.vision_bandwidth);
//	print_text("\nmanual_override_delay=");
//	print_number(truck.manual_override_delay);


	print_text("\nbattery_v0=");
	print_float(truck.battery_v0);
	print_text("\nsteering_step=");
	print_float(TO_DEG(truck.steering_step));
	print_text("\nsteering_overshoot=");
	print_float(TO_DEG(truck.steering_overshoot));
// 	print_text("\nstiction_threshold=");
// 	print_float(truck.stiction_threshold);
// 	print_text("\nstiction_amount=");
// 	print_float(truck.stiction_amount);

	print_text("\nSTEERING PID=");
	dump_pid(&truck.heading_pid);

	print_text("RPM PID=");
	dump_pid(&truck.rpm_pid);

// 	print_text("PATH PID=");
// 	dump_pid(&truck.path_pid);
// 
// 	print_text("SIDE PID=");
// 	dump_pid(&truck.side_pid);
}

int read_config_packet(const unsigned char *buffer)
{
	int offset = 0;

	offset += 4;
//	truck.vision_bandwidth = (float)buffer[offset++] / 0xff;
//	truck.vanish_center = buffer[offset++];
//	truck.bottom_center = buffer[offset++];
//	truck.enable_vision = buffer[offset++];
	truck.enable_mag = buffer[offset++];
	offset += 2;
//	truck.manual_override_delay = READ_UINT16(buffer, offset);

	truck.mid_steering100 = buffer[offset++];
	truck.mid_throttle100 = buffer[offset++];
	
	truck.max_throttle_fwd100 = buffer[offset++];
	truck.max_throttle_rev100 = buffer[offset++];
	
	truck.min_throttle_fwd100 = buffer[offset++];
	truck.min_throttle_rev100 = buffer[offset++];
	
	truck.throttle_base = buffer[offset++];
	truck.throttle_reverse_base = buffer[offset++];
	
	truck.max_steering100 = buffer[offset++];
	truck.min_steering100 = buffer[offset++];
	truck.auto_steering = buffer[offset++];
	truck.auto_throttle = buffer[offset++];
	truck.rpm_dv_size = buffer[offset++];
//	truck.path_dx_size = buffer[offset++];

	truck.gyro_center_max = READ_UINT16(buffer, offset);
	truck.max_gyro_drift = (float)READ_UINT16(buffer, offset) / 256;
	truck.angle_to_gyro = READ_INT16(buffer, offset);
	truck.imu.gyro_bandwidth = (float)buffer[offset++] / 0xff;
	truck.d_bandwidth = (float)buffer[offset++];



	truck.throttle_ramp_delay = READ_UINT16(buffer, offset);
	truck.throttle_ramp_step100 = READ_UINT16(buffer, offset);
	truck.pid_downsample = READ_UINT16(buffer, offset);
	truck.steering_step_delay = READ_UINT16(buffer, offset);
	
	
	truck.battery_analog = READ_UINT16(buffer, offset);
	truck.target_rpm = READ_UINT16(buffer, offset);
	truck.target_reverse_rpm = READ_UINT16(buffer, offset);
	
	
	truck.battery_v0 = READ_FLOAT32(buffer, offset);
	truck.steering_step = READ_FLOAT32(buffer, offset);
	truck.steering_overshoot = READ_FLOAT32(buffer, offset);
	
	
//TRACE
//print_buffer(buffer + offset, 4);
//	truck.stiction_threshold = READ_FLOAT32(buffer, offset);
//	truck.stiction_amount = READ_FLOAT32(buffer, offset);


// phone sends back the status packet here to have it saved
// 	truck.imu.mag_x_max = READ_INT16(buffer, offset);
// 	truck.imu.mag_y_max = READ_INT16(buffer, offset);
// 	truck.imu.mag_z_max = READ_INT16(buffer, offset);
// 	truck.imu.mag_x_min = READ_INT16(buffer, offset);
// 	truck.imu.mag_y_min = READ_INT16(buffer, offset);
// 	truck.imu.mag_z_min = READ_INT16(buffer, offset);

	offset = read_pid(&truck.heading_pid, buffer, offset);
	offset = read_pid(&truck.rpm_pid, buffer, offset);
//	offset = read_pid(&truck.path_pid, buffer, offset);
//	offset = read_pid(&truck.side_pid, buffer, offset);
	
	init_filter(&truck.d_filter, 0, truck.d_bandwidth / 100);
//	init_filter(&truck.d2_filter, 0, truck.d_bandwidth / 100);
	
//	truck.heading_pid.stiction_threshold = truck.stiction_threshold;
//	truck.heading_pid.stiction_amount = truck.stiction_amount;
	resize_derivative(&truck.rpm_dv, truck.rpm_dv_size);
//	resize_derivative(&truck.path_dx, truck.path_dx_size);

	truck.mid_steering_pwm = MIN_PWM + 
			(MAX_PWM - MIN_PWM) * 
			truck.mid_steering100 /
			100;
	truck.mid_throttle_pwm = MIN_PWM + 
			(MAX_PWM - MIN_PWM) * 
			truck.mid_throttle100 / 
			100;
	truck.min_throttle_reverse = truck.mid_throttle_pwm +
			(MAX_PWM - MIN_PWM) / 2 *
			truck.min_throttle_rev100 / 
			100;
	truck.min_throttle_fwd = truck.mid_throttle_pwm -
			(MAX_PWM - MIN_PWM) / 2 *
			truck.min_throttle_fwd100 / 
			100;
	truck.max_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.max_steering100 / 
			100;
	truck.min_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.min_steering100 / 
			100;
	truck.throttle_ramp_step = (MAX_PWM - MIN_PWM) *
			truck.throttle_ramp_step100 / 
			1000;

/*
 * TRACE2
 * print_text("size of config packet=");
 * print_number(offset);
 */


//	update_headlights();

// debug
//truck.max_throttle_fwd = 0;
//truck.max_throttle_rev = 0;
//truck.auto_steering = 1;

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
   	    USART_Cmd(USART3, DISABLE);
   	    USART_Cmd(USART1, DISABLE);

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


   	    USART_Cmd(USART3, ENABLE);
   	    USART_Cmd(USART1, ENABLE);

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

					radio_led_toggle();

// begin gyro calibration		
					if(truck.bt_throttle != 0 && 
						!truck.have_gyro_center && 
						!truck.need_gyro_center)
					{
#ifndef I2C_IMU
						begin_gyro_calibration();
#else
						calibrate_imu(&truck.imu);
#endif
					}

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

// calibrate mag
				if(truck.enable_mag)
				{
					if(receive_buf[11] &&
						!truck.imu.calibrate_mag)
					{
						calibrate_mag(&truck.imu);
					}
					else
// save mag settings
					if(!receive_buf[11] &&
						truck.imu.calibrate_mag)
					{
						truck.imu.calibrate_mag = 0;
					}
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
				WRITE_INT16(buf, offset, (int)truck.gyro_center);
				WRITE_INT16(buf, offset, truck.gyro_max - truck.gyro_min);
				WRITE_INT16(buf, offset, truck.rpm);
#ifdef I2C_IMU
				if(truck.enable_mag)
				{
					WRITE_FLOAT32(buf, offset, truck.imu.current_heading);
				}
				else
				
#endif
				{
					WRITE_FLOAT32(buf, offset, truck.current_heading);
				}

                WRITE_INT32(buf, offset, truck.throttle_avg);
                WRITE_INT16(buf, offset, truck.bluetooth_packets2);
                WRITE_INT16(buf, offset, truck.stick_packets2);
                buf[offset++] = truck.throttle_updated;
                truck.throttle_updated = 0;

// 				WRITE_INT16(buf, offset, truck.imu.mag_x_min);
// 				WRITE_INT16(buf, offset, truck.imu.mag_x_max);
// 				WRITE_INT16(buf, offset, truck.imu.mag_y_min);
// 				WRITE_INT16(buf, offset, truck.imu.mag_y_max);
// 				WRITE_INT16(buf, offset, truck.imu.mag_z_min);
// 				WRITE_INT16(buf, offset, truck.imu.mag_z_max);

// 				buf[offset++] = truck.vanish_x;
// 				buf[offset++] = truck.vanish_y;
// 				buf[offset++] = truck.bottom_x;


// 				WRITE_FLOAT32(buf, 
// 					offset, 
// 					truck.heading_pid.result);

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
				truck.need_gyro_center = 0;
				truck.gyro_center_count = 0;
				truck.gyro_center = 0;
				truck.gyro_accum = 0;
				truck.gyro_min = 0;
				truck.gyro_max = 0;
				truck.current_heading = 0;
// red
				SET_PIN(LED_GPIO2, LED_PIN2);
				CLEAR_PIN(LED_GPIO1, LED_PIN1);
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

					CLEAR_PIN(LED_GPIO1, LED_PIN1);
					SET_PIN(LED_GPIO2, LED_PIN2);
					int i;
					int prev_timer_h = truck.tick;
					int toggle = 0;
// stop the feedback loop
					truck.writing_settings = 1;

// wiggle the wheels & flash the LED if not driving
                    if(!truck.throttle)
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

						    TOGGLE_PIN(LED_GPIO1, LED_PIN1);
						    TOGGLE_PIN(LED_GPIO2, LED_PIN2);
					    }
                    }

					truck.writing_settings = 0;
					CLEAR_PIN(LED_GPIO1, LED_PIN1);
					SET_PIN(LED_GPIO2, LED_PIN2);
				}

				dump_config();
				break;
			}

		}
	}
}

static void get_code1();
static void get_code2();

static void get_data()
{
	truck.bluetooth.receive_buf[truck.bluetooth.receive_offset++] = 
		truck.bluetooth.data;
	if(truck.bluetooth.receive_offset >= truck.bluetooth.receive_size)
	{
		truck.bluetooth.got_data = 1;
		truck.bluetooth.current_function = get_code1;
	}
}

static void get_size()
{
	truck.bluetooth.receive_buf[truck.bluetooth.receive_offset++] = 
		truck.bluetooth.data;
	if(truck.bluetooth.receive_offset >= truck.bluetooth.receive_size)
	{
		truck.bluetooth.receive_size = truck.bluetooth.receive_buf[4] |
			(truck.bluetooth.receive_buf[5] << 8);
		if(truck.bluetooth.receive_size < BLUE_BUFSIZE)
		{
			truck.bluetooth.current_function = get_data;
		}
		else
		{
			truck.bluetooth.current_function = get_code1;
		}
	}
}

static void get_code4()
{
	if(truck.bluetooth.data == 0xe5)
	{
		truck.bluetooth.receive_buf[0] = 0xff;
		truck.bluetooth.receive_buf[1] = 0x2d;
		truck.bluetooth.receive_buf[2] = 0xd4;
		truck.bluetooth.receive_buf[3] = 0xe5;
		truck.bluetooth.current_function = get_size;
		truck.bluetooth.receive_offset = 4;
		truck.bluetooth.receive_size = 6;
	}
	else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = get_code1;
    }
}


static void get_code3()
{
	if(truck.bluetooth.data == 0xd4)
	{
    	truck.bluetooth.current_function = get_code4;
	}
    else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = get_code1;
    }
}


static void get_code2()
{
	if(truck.bluetooth.data == 0x2d)
    {
		truck.bluetooth.current_function = get_code3;
	}
    else
    if(truck.bluetooth.data == 0xff)
    {
        truck.bluetooth.current_function = get_code2;
    }
    else
	{
    	truck.bluetooth.current_function = get_code1;
    }
}


static void get_code1()
{
	if(truck.bluetooth.data == 0xff)
	{
    	truck.bluetooth.current_function = get_code2;
    }
}


void USART3_IRQHandler(void)
{
	truck.bluetooth.data = USART3->DR;
//    print_buffer(&truck.bluetooth.data, 1);
	truck.bluetooth.current_function();
}

// TIM10 wraps at TIMER_HZ
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
		(USART3->SR & USART_FLAG_TC) != 0)
	{
		USART3->DR = truck.bluetooth.send_buf[truck.bluetooth.send_offset++];
	}

	if(truck.bluetooth.got_data)
	{
		truck.bluetooth.got_data = 0;
		handle_beacon();
	}

#endif // !BLUETOOTH_PASSTHROUGH
}

void init_bluetooth()
{
    GPIO_InitTypeDef GPIO_InitStructure;
#ifdef BLUETOOTH_PASSTHROUGH
	truck.bluetooth.current_function = bluetooth_passthrough;
	TRACE2
	print_text("Entering bluetooth passthrough.\n");
#else
	truck.bluetooth.current_function = get_code1;
#endif


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#define UART_RX_PIN 11
#define UART_TX_PIN 10
	GPIO_PinAFConfig(GPIOB, UART_RX_PIN, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, UART_TX_PIN, GPIO_AF_USART3);
	
// RX enabled
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = 1 << UART_RX_PIN;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

// TX enabled
	GPIO_InitStructure.GPIO_Pin = 1 << UART_TX_PIN;
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

}

void handle_analog()
{
// battery
	if((ADC1->SR & ADC_FLAG_EOC))
	{
		truck.battery_accum += ADC1->DR;
		ADC_SoftwareStartConv(ADC1);
		truck.battery_count++;
		if(truck.battery_count >= BATTERY_OVERSAMPLE)
		{
			truck.battery = truck.battery_accum / truck.battery_count;
/*
 * TRACE2
 * print_number(truck.battery_accum);
 * print_number(truck.battery_count);
 * print_number(truck.battery);
 * print_float(truck.battery_voltage);
 */


			truck.battery_accum = 0;
			truck.battery_count = 0;
			truck.battery_voltage = (float)truck.battery * 
				truck.battery_v0 / 
				truck.battery_analog;



		}
	
	}


#ifndef I2C_IMU

// Analog gyro
	if((ADC2->SR & ADC_FLAG_EOC))
	{
		truck.gyro_accum += ADC2->DR;
		ADC_SoftwareStartConv(ADC2);
		truck.gyro_count++;
		if(truck.gyro_count >= GYRO_OVERSAMPLE)
		{
			DISABLE_INTERRUPTS


#ifdef REVERSE_GYRO
			truck.gyro = (float)truck.ref - 
                truck.gyro_accum / truck.gyro_count;
#else
			truck.gyro = (float)truck.gyro_accum / truck.gyro_count -
				truck.ref;
#endif

			ENABLE_INTERRUPTS
			
			truck.gyro_accum = 0;
			truck.gyro_count = 0;
//TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);





//			TRACE2
//			print_number(gyro);
			
			
			if(truck.need_gyro_center && !truck.have_gyro_center)
			{
				imu_led_flash();


				truck.gyro_center_accum += truck.gyro;
				if(truck.gyro_center_count == 0)
				{
					truck.gyro_min = truck.gyro;
					truck.gyro_max = truck.gyro;
				}
				else
				{
					truck.gyro_min = MIN(truck.gyro, truck.gyro_min);
					truck.gyro_max = MAX(truck.gyro, truck.gyro_max);
				}
				truck.gyro_center_count++;

				if(ABS(truck.gyro_max - truck.gyro_min) > truck.gyro_center_max)
				{
					TRACE2
					print_text("center too big min=");
					print_number(truck.gyro_min);
					print_text("max=");
					print_number(truck.gyro_max);
					print_lf();
					truck.gyro_center_count = 0;
					truck.gyro_center_accum = 0;
					truck.gyro_min = 0;
					truck.gyro_max = 0;
				}
			}

			if(truck.need_gyro_center &&
				!truck.have_gyro_center && 
				truck.gyro_center_count >= GYRO_CENTER_TOTAL)
			{
				truck.prev_gyro_center = truck.gyro_center;
				truck.gyro_center = (float)truck.gyro_center_accum / 
					truck.gyro_center_count;

				TRACE2
				print_text("center=");
				print_float(truck.gyro_center);
				print_text("drift=");
				print_float(truck.prev_gyro_center - truck.gyro_center);

// test if calculation didn't drift
				if(fabs(truck.prev_gyro_center) > 0 &&
					fabs(truck.prev_gyro_center - truck.gyro_center) <
						(float)truck.max_gyro_drift)
				{
					TRACE2
					print_text("done");
					truck.have_gyro_center = 1;
					truck.need_gyro_center = 0;
				}
				else
// do another calculation
				{
					truck.gyro_center_accum = 0;
					truck.gyro_center_count = 0;
					truck.gyro_accum = 0;
					truck.gyro_min = 0;
					truck.gyro_max = 0;
				}
			}

			if(truck.have_gyro_center)
			{
				DISABLE_INTERRUPTS
                float raw_gyro = (float)(truck.gyro - truck.gyro_center) / 
					truck.angle_to_gyro /
					NAV_HZ;
				truck.current_heading += raw_gyro;



				truck.current_heading = fix_angle(truck.current_heading);
				ENABLE_INTERRUPTS
				
				truck.debug_counter++;
				if(truck.debug_counter >= 128)
				{
					truck.debug_counter = 0;
//					TRACE2
//					print_float(truck.gyro);
//					print_float(TO_DEG(truck.current_heading));
				}
			}
		}
	}


	if((ADC3->SR & ADC_FLAG_EOC))
	{
		truck.ref_accum += ADC3->DR;

		ADC_SoftwareStartConv(ADC3);

		truck.ref_count++;
		if(truck.ref_count >= GYRO_OVERSAMPLE)
		{
			DISABLE_INTERRUPTS
			truck.ref = (float)truck.ref_accum / truck.ref_count;
			ENABLE_INTERRUPTS

			truck.ref_count = 0;
			truck.ref_accum = 0;


/*
 * 			truck.debug_counter++;
 * 			if(truck.debug_counter >= 128)
 * 			{
 * 				truck.debug_counter = 0;
 * 				TRACE2
 * 				print_float(truck.ref);
 * 			}
 */
		}
	}
#endif //  !I2C_IMU




}


void init_analog()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
// The battery
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

#ifndef I2C_IMU
// The analog gyro
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
// Current sensor/ gyro ref
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
#endif

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


#ifndef I2C_IMU
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);
#endif

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;

#ifndef I2C_IMU

	GPIO_InitStructure.GPIO_Pin |=
		GPIO_Pin_1 |
		GPIO_Pin_2;
#endif

	GPIO_Init(GPIOA, &GPIO_InitStructure);

#ifndef I2C_IMU

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

#endif

// must call this before ENABLE
	ADC_RegularChannelConfig(ADC1, 
		ADC_Channel_0, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);
	ADC_SoftwareStartConv(ADC1);


#ifndef I2C_IMU
// The gyro channel
	ADC_RegularChannelConfig(ADC2, 
		ADC_Channel_1, 
		1, 
		ADC_SampleTime_480Cycles);
// The ref voltage channel
	ADC_RegularChannelConfig(ADC3, 
		ADC_Channel_2, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC2);
	ADC_SoftwareStartConv(ADC3);
#endif

	

}


void steering_input_throttle(float *steering_overshoot)
{
	switch(truck.steering)
	{
// full left
		case 1:
			truck.steering_pwm = truck.mid_steering_pwm - truck.max_steering_magnitude;
			*steering_overshoot = truck.steering_overshoot;
			truck.need_steering_feedback = 0;
			break;

// slow left
		case 2:
			if(!truck.auto_steering ||
				truck.steering_first ||
				truck.throttle_reverse)
			{
// continue steering from before throttle
				truck.steering_pwm = truck.mid_steering_pwm - truck.min_steering_magnitude;
                *steering_overshoot = 0;
				truck.need_steering_feedback = 0;
			}
			else
			{
				truck.need_steering_feedback = 1;
				truck.steering_step_counter++;
				if(truck.steering_step_counter >= 
					truck.steering_step_delay * PWM_HZ / PWM_BASE)
				{
					truck.steering_step_counter = 0;
					truck.target_heading -= truck.steering_step;
					truck.target_heading = fix_angle(truck.target_heading);
//TRACE2
//print_float(TO_DEG(truck.current_heading));
				}
			}
			break;


// slow right
		case 3:
			if(!truck.auto_steering ||
				truck.steering_first ||
				truck.throttle_reverse)
			{
// continue steering from before throttle
				truck.steering_pwm = truck.mid_steering_pwm + truck.min_steering_magnitude;
                *steering_overshoot = 0;
				truck.need_steering_feedback = 0;
			}
			else
			{
				truck.need_steering_feedback = 1;
				truck.steering_step_counter++;
				if(truck.steering_step_counter >= 
					truck.steering_step_delay * PWM_HZ / PWM_BASE)
				{
					truck.steering_step_counter = 0;
					truck.target_heading += truck.steering_step;
					truck.target_heading = fix_angle(truck.target_heading);
//TRACE2
//print_float(TO_DEG(truck.current_heading));
				}
			}
			break;


// full right
		case 4:
			truck.steering_pwm = truck.mid_steering_pwm + truck.max_steering_magnitude;
			*steering_overshoot = -truck.steering_overshoot;
			truck.need_steering_feedback = 0;
			break;


		default:
// Force next steering press to engage heading hold
			truck.steering_first = 0;
			truck.need_steering_feedback = 1;


			if(!truck.auto_steering)
			{
				truck.need_steering_feedback = 0;
				truck.steering_pwm = truck.mid_steering_pwm;
			}
			break;
	}
}

void steering_input_nothrottle()
{
	switch(truck.steering)
	{
// full left
		case 1:
			truck.steering_pwm = truck.mid_steering_pwm - truck.max_steering_magnitude;
			truck.steering_first = 1;
			truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
			break;
// slow left
		case 2:
			truck.steering_pwm = truck.mid_steering_pwm - truck.min_steering_magnitude;
			truck.steering_first = 1;
			truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
			break;
// slow right
		case 3:
			truck.steering_pwm = truck.mid_steering_pwm + truck.min_steering_magnitude;
			truck.steering_first = 1;
			truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
			break;
// full right
		case 4:
			truck.steering_pwm = truck.mid_steering_pwm + truck.max_steering_magnitude;
			truck.steering_first = 1;
			truck.need_steering_feedback = 0;
            truck.steering_timeout = 0;
			break;
// no steering command
		default:
			truck.steering_pwm = truck.mid_steering_pwm;
			truck.steering_first = 0;
			break;
	}
}

void handle_steering_feedback(int steering_feedback100)
{
	if(truck.steering_timeout > 0)
	{
		truck.steering_timeout--;
	}
	else
	{
// disable heading hold after a certain time with no throttle or a change in reverse
		truck.need_steering_feedback = 0;
	}					

#ifndef I2C_IMU
	int raw_gyro = truck.gyro - truck.gyro_center;
#else // !I2C_IMU
	int raw_gyro = truck.imu.gyro_z_centered;
#endif

// filtered heading errors
	float error = -get_angle_change(truck.current_heading, truck.target_heading);
	float p = error;
	float d = do_highpass(&truck.d_filter, error);
//					float d2 = do_highpass(&truck.d2_filter, (float)raw_gyro / truck.angle_to_gyro);
	float d2 = (float)raw_gyro / truck.angle_to_gyro;



	steering_feedback100 = do_pid(&truck.heading_pid, 
		p, 
		d, 
		d2);


	truck.steering_pwm = truck.mid_steering_pwm -
		steering_feedback100 * 
		(MAX_PWM - MIN_PWM) / 2 / 
		100;

// print_text("steering_pwm=");
// print_number(truck.steering_pwm);
// print_lf();
}


void handle_manual_steering(int steering_overshoot)
{

    reset_filter(&truck.d_filter);
	reset_pid(&truck.heading_pid);


#ifdef I2C_IMU
// 	if(truck.enable_mag)
// 	{
// 		truck.target_heading = truck.imu.current_heading + 
// 			steering_overshoot;
// 	}
// 	else
#endif
	{
		truck.target_heading = truck.current_heading +
            steering_overshoot;
	}

    truck.target_heading = fix_angle(truck.target_heading);
}


void handle_auto_throttle(int throttle_base)
{
    int target_rpm = 0;
	if(truck.throttle_reverse)
	{
		target_rpm = truck.target_reverse_rpm;
	}
	else
	{
		target_rpm = truck.target_rpm;
	}

	truck.throttle_feedback = do_pid(&truck.rpm_pid,
		(float)(target_rpm - truck.rpm) / 1000,
		-get_derivative(&truck.rpm_dv) / 1000,
		0);

	if(truck.throttle_reverse)
	{
		truck.throttle_pwm = truck.mid_throttle_pwm + 
			(MAX_PWM - MIN_PWM) / 2 *
			(throttle_base + truck.throttle_feedback) /
			100;
// don't go into braking region
		if(truck.throttle_pwm < truck.min_throttle_reverse)
		{
			truck.rpm_pid.ignore_i = 1;
			truck.throttle_pwm = truck.min_throttle_reverse;
		}
	}
	else
	{
		truck.throttle_pwm = truck.mid_throttle_pwm - 
			(MAX_PWM - MIN_PWM) / 2 *
			(throttle_base + truck.throttle_feedback) /
			100;
// don't go into braking region
		if(truck.throttle_pwm > truck.min_throttle_fwd)
		{
			truck.rpm_pid.ignore_i = 1;
			truck.throttle_pwm = truck.min_throttle_fwd;
		}
	}
}


void handle_throttle_ramp(int throttle_base, int throttle_magnitude)
{
	truck.throttle_ramp_counter++;
	if(truck.throttle_ramp_counter >= 
		truck.throttle_ramp_delay * PWM_HZ / PWM_BASE)
	{
		truck.throttle_ramp_counter = 0;


// get target PWM based on throttle_base
		if(truck.auto_throttle)
		{
			throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
				throttle_base / 
				100;
		}

		if(truck.throttle_reverse)
		{
			if(truck.throttle_pwm < truck.mid_throttle_pwm +
					throttle_magnitude)
			{
				truck.throttle_pwm += truck.throttle_ramp_step;
			}
			else
			{
				truck.throttle_state = THROTTLE_WAIT;
				truck.start_time = truck.tick;
			}

			truck.throttle_pwm = MIN(truck.mid_throttle_pwm +
				throttle_magnitude,
				truck.throttle_pwm);
		}
		else
		{

			if(truck.throttle_pwm > truck.mid_throttle_pwm -
					throttle_magnitude)
			{
				truck.throttle_pwm -= truck.throttle_ramp_step;
			}
			else
			{
				truck.throttle_state = THROTTLE_WAIT;
				truck.start_time = truck.tick;
			}

			truck.throttle_pwm = MAX(truck.mid_throttle_pwm -
				throttle_magnitude,
				truck.throttle_pwm);
		}
	}

}

void handle_throttle_change()
{
	truck.throttle_state = THROTTLE_RAMP;

// heading is reset after a timeout or steering command after stopping
    if(truck.steering_timeout <= 0)
    {
    	truck.current_heading = 0;
        truck.target_heading = 0;
    }


	if(truck.throttle_reverse)
	{
		truck.throttle_pwm = truck.min_throttle_reverse;
	}
	else
	{
		truck.throttle_pwm = truck.min_throttle_fwd;
	}


// #ifdef I2C_IMU
// 	if(truck.enable_mag)
// 	{
// 		truck.target_heading = truck.imu.current_heading;
// 	}
// 	else
// #endif
// 	{
// 		truck.target_heading = 0;
// 	}

	truck.throttle_ramp_counter = 0;
	reset_pid(&truck.heading_pid);
	reset_pid(&truck.rpm_pid);
//	reset_pid(&truck.path_pid);
//	reset_pid(&truck.side_pid);
}



void handle_bt_input(int throttle_magnitude)
{
// always manual steering
	truck.need_steering_feedback = 0;
	int target_throttle = truck.mid_throttle_pwm -
		throttle_magnitude * 
		truck.bt_throttle / 
		127;
// forward
	if(truck.bt_throttle > 0)
	{
// decrease PWM to target
		if(truck.throttle_pwm > target_throttle)
		{
			truck.throttle_ramp_counter++;
			if(truck.throttle_ramp_counter >= 
				truck.throttle_ramp_delay * PWM_HZ / PWM_BASE)
			{
				truck.throttle_ramp_counter = 0;
				truck.throttle_pwm -= truck.throttle_ramp_step;
				truck.throttle_pwm = MAX(truck.throttle_pwm, 
					truck.mid_throttle_pwm -
					throttle_magnitude);
			}
		}
		else
		{
			truck.throttle_pwm = target_throttle;
		}
	}
	else
// reverse
	if(truck.bt_throttle < 0)
	{
// increase PWM to target
		if(truck.throttle_pwm < target_throttle)
		{
			truck.throttle_ramp_counter++;
			if(truck.throttle_ramp_counter >= 
				truck.throttle_ramp_delay * PWM_HZ / PWM_BASE)
			{
				truck.throttle_ramp_counter = 0;
				truck.throttle_pwm += truck.throttle_ramp_step;
				truck.throttle_pwm = MIN(truck.throttle_pwm, 
					truck.mid_throttle_pwm +
					throttle_magnitude);
			}
		}
		else
		{
			truck.throttle_pwm = target_throttle;
		}
	}
	else
	{
// no bluetooth throttle
		truck.throttle_pwm = truck.mid_throttle_pwm;
	}


	truck.steering_pwm = truck.mid_steering_pwm +
		truck.max_steering_magnitude *
		truck.bt_steering / 
		127;
}




// the mane feedback loop fires for every PWM period
void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_FLAG_Update)
	{
		TIM2->SR = ~TIM_FLAG_Update;
#ifdef USE_BRUSHLESS
		SET_PIN(GPIOA, ESC_PIN);
#endif
		SET_PIN(GPIOC, GPIO_Pin_4);

		if(truck.writing_settings) return;

// -100 - 100
		float steering_feedback100 = 0;
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
				truck.throttle = 0;
				truck.steering = 0;
			}

// Change in throttle
			if((truck.throttle > 0 && truck.throttle2 <= 0) ||
				(truck.throttle < 0 && truck.throttle2 >= 0))
			{
                handle_throttle_change();
			}

// change in steering
			if(truck.steering != truck.steering2)
			{
				truck.throttle_ramp_counter = 0;
				truck.steering_step_counter = 0;
			}

// change in direction
			if(truck.throttle_reverse2 != truck.throttle_reverse)
			{
				truck.steering_timeout = 0;
			}
			
			truck.steering2 = truck.steering;
			truck.throttle2 = truck.throttle;
			truck.throttle_reverse2 = truck.throttle_reverse;


			int throttle_magnitude = 0;
			if(truck.have_bt_controls && truck.bt_throttle < 0)
			{
				throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
					truck.max_throttle_rev100 / 
					100;
// don't use steering feedback in reverse
				truck.need_steering_feedback = 0;
			}
			else
			{
				throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
					truck.max_throttle_fwd100 / 
					100;
			}

			if(truck.have_bt_controls)
			{
                handle_bt_input(throttle_magnitude);
			}
			else
// stick controller
			if(truck.throttle > 0)
			{

				truck.steering_timeout = STEERING_RELOAD;

				int throttle_base;
				if(truck.throttle_reverse)
				{
					throttle_base = truck.throttle_reverse_base;
				}
				else
				{
					throttle_base = truck.throttle_base;
				}

				if(truck.throttle_state == THROTTLE_RAMP)
				{
                    handle_throttle_ramp(throttle_base, throttle_magnitude);
// TRACE2
// print_text("throttle_pwm=");
// print_number(truck.throttle_pwm);

				}
				else
				if(truck.throttle_state == THROTTLE_WAIT)
				{
					if(truck.tick - truck.start_time >= 
						1 * TIMER_HZ)
					{
						truck.throttle_state = THROTTLE_AUTO;
						truck.start_time = truck.tick;
					}
				}
				else
// THROTTLE_AUTO
				{
					if(truck.auto_throttle)
					{

                        handle_auto_throttle(throttle_base);

					}
					else
					{
					
						if(truck.throttle_reverse)
						{
							truck.throttle_pwm = truck.mid_throttle_pwm - 
								(MAX_PWM - MIN_PWM) / 2 *
								truck.max_throttle_rev100 /
								100;
						}
						else
						{
							truck.throttle_pwm = truck.mid_throttle_pwm - 
								(MAX_PWM - MIN_PWM) / 2 *
								truck.max_throttle_fwd100 /
								100;
						}
					}
				}

                steering_input_throttle(&steering_overshoot);
			}
			else
// steering with no throttle
			{
				truck.throttle_pwm = truck.mid_throttle_pwm;
                
                steering_input_nothrottle();
			}


// TRACE2
// print_text("throttle_state=");
// print_number(truck.throttle_state);
// print_text(" truck.mid_throttle_pwm=");
// print_float(truck.mid_throttle_pwm);
// print_text(" truck.throttle_pwm=");
// print_number(truck.throttle_pwm);
// }

			if(truck.need_steering_feedback)
			{
                handle_steering_feedback(steering_feedback100);

			}
			else
			{
                handle_manual_steering(steering_overshoot);
			}

/*
 *          TRACE2
 *          print_float(steering_feedback100);
 */



// send debug info to the Odroid running video_debug.c
// unsigned char debug_buffer[256];
// int offset = 0;
// debug_buffer[offset++] = 0xff;
// debug_buffer[offset++] = SYNC_CODE;
// WRITE_FLOAT32(debug_buffer, offset, truck.current_heading);
// WRITE_FLOAT32(debug_buffer, offset, truck.target_heading);
// WRITE_FLOAT32(debug_buffer, offset, truck.heading_pid.p_input);
// WRITE_FLOAT32(debug_buffer, offset, truck.heading_pid.d_input);
// WRITE_FLOAT32(debug_buffer, offset, truck.heading_pid.d2_input);
// WRITE_FLOAT32(debug_buffer, offset, steering_feedback100);
// send_uart_binary(debug_buffer, offset);



            truck.throttle_accum += truck.throttle_pwm;
            truck.throttle_count++;


			write_pwm();
		} // have_gyro_center
		else
		{
			truck.throttle_pwm = truck.mid_throttle_pwm;
			truck.steering_pwm = truck.mid_steering_pwm;
			
			write_pwm();
		}
	}

	if(TIM2->SR & TIM_FLAG_CC1)
	{
		TIM2->SR = ~TIM_FLAG_CC1;
#ifdef USE_BRUSHLESS
		CLEAR_PIN(GPIOA, ESC_PIN);
#endif
	}

	if(TIM2->SR & TIM_FLAG_CC2)
	{
		TIM2->SR = ~TIM_FLAG_CC2;
		CLEAR_PIN(GPIOC, GPIO_Pin_4);
	}
}

void write_pwm()
{
//print_number(truck.throttle_pwm);
//print_lf();
#ifdef REVERSE_STEERING
	int mid_steering_pwm = (MIN_PWM + MAX_PWM) / 2;
	truck.steering_pwm = mid_steering_pwm - 
		(truck.steering_pwm - mid_steering_pwm);
#endif

#ifndef USE_BRUSHLESS
/*
 * 	int mid_throttle_pwm = MIN_PWM + 
 * 		(MAX_PWM - MIN_PWM) * 
 * 		truck.mid_throttle100 / 
 * 		100;
 */

// Bluetooth reverse is based on positive PWM
	if(truck.throttle_pwm > truck.mid_throttle_pwm)
	{
		int pwm = (truck.throttle_pwm - truck.mid_throttle_pwm) * 
			MAX_PERIOD /
			((MAX_PWM - MIN_PWM) / 2);
// strange artifact makes it die if below a certain amount
		pwm = MAX(pwm, 4000);
		SET_COMPARE(TIM3, CCR2, MAX_PERIOD - pwm);
		SET_COMPARE(TIM3, CCR1, MAX_PERIOD);
	}
	else
// Stick reverse & forward is based on negative PWM
	if(truck.throttle_pwm < truck.mid_throttle_pwm)
	{
		int pwm = (truck.mid_throttle_pwm - truck.throttle_pwm) * 
			MAX_PERIOD /
			((MAX_PWM - MIN_PWM) / 2);
		pwm = MAX(pwm, 4000);
		if(truck.throttle_reverse)
		{
			SET_COMPARE(TIM3, CCR2, MAX_PERIOD - pwm);
			SET_COMPARE(TIM3, CCR1, MAX_PERIOD);
		}
		else
		{
			SET_COMPARE(TIM3, CCR2, MAX_PERIOD);
			SET_COMPARE(TIM3, CCR1, MAX_PERIOD - pwm);
		}
	}
	else
	{
		SET_COMPARE(TIM3, CCR2, MAX_PERIOD);
		SET_COMPARE(TIM3, CCR1, MAX_PERIOD);
	}
#else // !USE_BRUSHLESS


	CLAMP(truck.steering_pwm, MIN_PWM, MAX_PWM);
	CLAMP(truck.throttle_pwm, MIN_PWM, MAX_PWM);
	SET_COMPARE(TIM2, CCR1, truck.throttle_pwm);
	SET_COMPARE(TIM2, CCR2, truck.steering_pwm);



#endif // USE_BRUSHLESS
}

void init_pwm()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_ResetBits(GPIOC, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

#ifdef USE_BRUSHLESS
	GPIO_InitStructure.GPIO_Pin = ESC_PIN;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif



	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	TIM_TimeBaseStructure.TIM_Period = PWM_PERIOD;
// Seems to be a power of 2
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = truck.throttle_pwm;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = truck.steering_pwm;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  	TIM_Cmd(TIM2, ENABLE);

#ifndef USE_BRUSHLESS
	GPIO_InitStructure.GPIO_Pin = ESC_PIN;
	GPIO_InitStructure.GPIO_Pin |= GPIO_Pin_7;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6,  GPIO_AF_TIM3);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_TimeBaseStructure.TIM_Period = MAX_PERIOD;
// Seems to be a power of 2
	TIM_TimeBaseStructure.TIM_Prescaler = 2;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  	TIM_Cmd(TIM3, ENABLE);
 	TIM_CtrlPWMOutputs(TIM3, ENABLE);
#endif // !USE_BRUSHLESS


 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, TIM_IT_Update | TIM_IT_CC1 | TIM_IT_CC2, ENABLE);
	
}


void handle_rpm()
{
/*
 * TRACE2
 * print_number(PIN_IS_SET(RPM_GPIO, RPM_PIN));
 */
// Each revolution is 2 crossings of the magnet
	if(truck.rpm_status)
	{
		if(!PIN_IS_SET(RPM_GPIO, RPM_PIN))
		{
			truck.rpm_status = 0;
			truck.rpm_counter++;
		}
	}
	else
	{
		if(PIN_IS_SET(RPM_GPIO, RPM_PIN))
		{
			truck.rpm_status = 1;
			truck.rpm_counter++;
		}
	}


// update the rpm after a certain time
	DISABLE_INTERRUPTS
	if(truck.tick - truck.rpm_time >= TIMER_HZ / 10)
	{
		truck.rpm_time = truck.tick;
		truck.rpm = truck.rpm_counter * 300 / 15;
		truck.rpm_counter = 0;
		ENABLE_INTERRUPTS


		update_derivative(&truck.rpm_dv, truck.rpm);

//		if(truck.throttle > 0)
		{
// 			TRACE2
// 			print_text("rpm=");
// 			print_number(truck.rpm);
// 			print_text("pwm=");
// 			print_number(truck.throttle_pwm);

/*
 * 			print_text("{ ");
 * 			print_number_nospace((int)truck.battery_voltage);
 * 			print_text(".");
 * 			int fraction = (int)(truck.battery_voltage * 100) % 100;
 * 			print_number_nospace(fraction / 10);
 * 			print_number_nospace(fraction % 10);
 * 			print_text("f, ");
 * 			print_number_nospace((int)truck.power);
 * 			print_text(", ");
 * 			print_number_nospace(truck.rpm);
 * 			print_text(" },\n");
 */

/*
 * 			print_text("V=");
 * 			print_float(truck.battery_voltage);
 * 			print_text("PWM=");
 * 			print_number(pwm);
 * 			print_text("RPM=");
 * 			print_number(truck.rpm_counter);
 */
		}
	}
	else
	{
		ENABLE_INTERRUPTS
	}
}



#ifdef SPI_NAV
// navigation data from SPI
void init_spi()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = MOSI_PIN | CLK_PIN;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource4);
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line = EXTI_Line4;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	truck.spi_state = SPI_SYNC_CODE;
}


void EXTI4_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line4) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line4);

    switch(truck.spi_state)
	{
		case SPI_SYNC_CODE:
			truck.spi_buffer <<= 1;
			truck.spi_buffer |= PIN_IS_SET(SPI_GPIO, MOSI_PIN);
			if(truck.spi_buffer == 0xff2dd4e5)
			{
				truck.spi_buffer = 0;
				truck.spi_state = SPI_PACKET;
				truck.spi_counter = 0;
			}
			break;

		case SPI_PACKET:
			truck.spi_buffer <<= 1;
			truck.spi_buffer |= PIN_IS_SET(SPI_GPIO, MOSI_PIN);
			truck.spi_counter++;
			if(truck.spi_counter >= 32)
			{
				truck.have_spi = 1;
				truck.spi_state = SPI_SYNC_CODE;

//print_hex(truck.spi_buffer);
				truck.vanish_x = (truck.spi_buffer >> 24) & 0xff;
				truck.vanish_y = (truck.spi_buffer >> 16) & 0xff;
				truck.bottom_x = (truck.spi_buffer >> 8) & 0xff;
//				update_derivative(&truck.path_dx, truck.path_x);
				truck.spi_buffer = 0;
			}
			break;
	}
  }
}

void handle_spi()
{
	if(truck.have_spi)
	{
		DISABLE_INTERRUPTS
// 		truck.vanish_lowpass = truck.vanish_x * truck.vision_bandwidth +
// 			truck.vanish_lowpass * (1.0f - truck.vision_bandwidth);
// 		truck.bottom_lowpass = truck.bottom_x * truck.vision_bandwidth +
// 			truck.bottom_lowpass * (1.0f - truck.vision_bandwidth);
		ENABLE_INTERRUPTS

		TRACE
		print_float(truck.vanish_lowpass);
// 		print_number(truck.vanish_x);
// 		print_number(truck.vanish_y);
// 		print_number(truck.bottom_x);
		truck.have_spi = 0;
	}
}


#endif // SPI_NAV





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

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();
	
	truck.steering = STEERING_MID;


	truck.headlights_on = 0;
	truck.mid_steering100 = 50;
	truck.mid_throttle100 = 50;
	truck.max_throttle_fwd100 = 100;
	truck.max_throttle_rev100 = 100;
	truck.max_steering100 = 100;
	truck.min_steering100 = 25;
	truck.auto_steering = 1;
	truck.auto_throttle = 1;
	
	truck.gyro_center_max = 100;
	truck.max_gyro_drift = 0.1f;
	truck.angle_to_gyro = 450;
	truck.throttle_ramp_delay = 0;
	truck.throttle_ramp_step100 = 1;
	
	
	
//	truck.target_power = 15.0f;
//	truck.power_base = 20.0f;
//	truck.rpm_slope = 0;
	
	
	
	truck.target_rpm = 500;
	truck.pid_downsample = 1;
	truck.steering_step_delay = 0;
	truck.steering_step = TO_RAD(30) / PWM_HZ;
	truck.steering_overshoot = 0;
//	truck.stiction_threshold = 0;
//	truck.stiction_amount = 0;
	
	truck.battery_analog = 965;
	truck.battery_v0 = 8.67f;

// heading error -> PWM

	init_pid(&truck.heading_pid, 0, 0, 0, 0, 0, 0, 0);
	init_filter(&truck.p_filter, 0, 1);
	init_filter(&truck.d_filter, 0, 0);
//	init_filter(&truck.d2_filter, 0, 0);


// RPM error -> throttle
	init_pid(&truck.rpm_pid, 0, 0, 0, 0, 0, 0, 0);

// vanishing point error -> heading
//	init_pid(&truck.path_pid, 0, 0, 0, 0, 0, 0);

// side error -> heading
//	init_pid(&truck.side_pid, 0, 0, 0, 0, 0, 0);

	truck.rpm_dv_size = 10;
	init_derivative(&truck.rpm_dv, truck.rpm_dv_size);
//	truck.path_dx_size = 9;
//	init_derivative(&truck.path_dx, truck.path_dx_size);


// the real settings are loaded here
	load_config();
	dump_config();


	truck.throttle_pwm = MIN_PWM + (MAX_PWM - MIN_PWM) * truck.mid_throttle100 / 100;
	truck.steering_pwm = MIN_PWM + (MAX_PWM - MIN_PWM) * truck.mid_steering100 / 100;


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

	print_text("Welcome to truck\n");
	flush_uart();

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

	GPIO_InitStructure.GPIO_Pin = LED_PIN1;
	GPIO_Init(LED_GPIO1, &GPIO_InitStructure);
	CLEAR_PIN(LED_GPIO1, LED_PIN1);

	GPIO_InitStructure.GPIO_Pin = LED_PIN2;
	GPIO_Init(LED_GPIO2, &GPIO_InitStructure);
	SET_PIN(LED_GPIO2, LED_PIN2);

// debug pin
	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
	GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);
	CLEAR_PIN(DEBUG_GPIO, DEBUG_PIN);

//	GPIO_InitStructure.GPIO_Pin = HEADLIGHT_PIN;
//	GPIO_Init(HEADLIGHT_GPIO, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
// ESC is connected via a diode to prevent explosion if it's plugged into a servo signal

#ifndef IS_CAR
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
#else
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
#endif


	GPIO_InitStructure.GPIO_Pin = RPM_PIN;
	GPIO_Init(RPM_GPIO, &GPIO_InitStructure);
	truck.rpm_time = truck.tick;
	truck.rpm_status = PIN_IS_SET(RPM_GPIO, RPM_PIN);





//	update_headlights();

	init_bluetooth();
	init_analog();
	init_pwm();

#ifdef I2C_IMU
	init_imu(&truck.imu);
#endif

#ifdef SPI_NAV
	init_spi();
#endif


#ifdef UART_NAV
	init_nav_uart();
#endif


#ifdef USE_CC1101
	init_cc1101();
	cc1101_receiver();
#endif

#ifdef USE_XBEE
    init_xbee();
#endif

/*
 * 	TRACE2
 * 	print_text("\nimu.angle_to_gyro=");
 * 	print_number(truck.imu.angle_to_gyro);
 */


	while(1)
	{

		handle_uart();
		handle_bluetooth();
		handle_analog();
		handle_rpm();

#ifdef SPI_NAV
		handle_spi();
#endif


#ifdef I2C_IMU
		HANDLE_IMU(truck.imu)
#endif


		if(radio.got_packet)
		{
			radio.got_packet = 0;
			handle_radio();
		}





	}
	
		
}





