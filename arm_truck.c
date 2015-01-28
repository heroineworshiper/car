/*
 * 1 handed truck
 * Copyright (C) 2012-2015 Adam Williams <broadcast at earthling dot net>
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

// make;../copter/uart_programmer truck.bin

// pass bluetooth to the debug port to configure the device by enabling 
// BLUETOOTH_PASSTHROUGH
// useful configuration commands:
// at+version
// AT+VERSION
// at+nametruck
// AT+NAMEtruck
// at+baud8
// AT+BAUD8
// new devices start at 9600 baud.  Be sure to set the initial baud rate 
// to 9600, then 115200 after configuration.
// Some kind of delay in the terminal program is required to paste text in,
// but the entire command must be sent in under a second.



#include "arm_truck.h"
#include "cc1101.h"
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


// pass bluetooth to debug port
//#define BLUETOOTH_PASSTHROUGH
#define SYNC_CODE 0xe5

// Address of persistent settings
#define SETTINGS_ADDRESS 0x0800c000
// Magic number for settings
#define SETTINGS_MAGIC 0x10291976

#define DEBUG_PIN GPIO_Pin_4
#define DEBUG_GPIO GPIOB

#define RPM_PIN GPIO_Pin_0
#define RPM_GPIO GPIOB

#define SPI_GPIO GPIOB
#define MOSI_PIN GPIO_Pin_3
#define CLK_PIN GPIO_Pin_4

//#define HEADLIGHT_PIN GPIO_Pin_13
//#define HEADLIGHT_GPIO GPIOC

// packets per second
#define PACKET_RATE 40

#define THROTTLE_MAX 0x1
// analog range
#define STEERING_MID 0x0


#define BATTERY_OVERSAMPLE 10000
#define GYRO_OVERSAMPLE 64
#define GYRO_CENTER_TOTAL (NAV_HZ * 5)
#define CURRENT_OVERSAMPLE 5000

// TIM10 wraps at this frequency
#define TIMER_HZ 100
#define TIMEOUT_RELOAD TIMER_HZ * 5
// gyro update rate
#define NAV_HZ 320
// PWM frequency
#define PWM_HZ 50
// timer for LED flashing
#define LED_DELAY (NAV_HZ / 2)
// ADC for 0 current
#define CURRENT_BASE 120.0f
// scale factor for current
#define CURRENT_SCALE 630.0f

// 50hz
#define PWM_PERIOD 1680000
// 2ms
#define MAX_PWM 168000
// 1ms
#define MIN_PWM 84000

#define MAX_PERIOD 65535

//#define DO_THROTTLE_TEST
#define THROTTLE_POWER1 10
#define THROTTLE_RPM2 350
#define POWER_STEP 5
#define THROTTLE_DELAY 5

//#define DO_RPM_TEST
#define RPM1 300
#define RPM2 700

//#define POWER_FEEDBACK
#define RPM_FEEDBACK
#define POWER_CURVE

//#define DO_PWM_TEST
#define PWM1 0
#define PWM2 20
#define TEST_TIME (5 * TIMER_HZ * 60)

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
	float i_limit,
	float o_limit)
{
	pid->p_gain = p_gain;
	pid->i_gain = i_gain;
	pid->d_gain = d_gain;
	pid->i_limit = i_limit;
	pid->o_limit = o_limit;
}

void reset_pid(pid_t *pid)
{
	pid->error_accum = 0;
	pid->accum = 0;
	pid->counter = 0;
}


int read_pid(pid_t *pid, const unsigned char *buffer, int offset)
{
	float p = READ_FLOAT32(buffer, offset);
	float i = READ_FLOAT32(buffer, offset);
	float d = READ_FLOAT32(buffer, offset);
	float i_limit = READ_FLOAT32(buffer, offset);
	float o_limit = READ_FLOAT32(buffer, offset);
	init_pid(pid, 
		p, // P gain
		i, // I gain	
		d, // D gain
		i_limit, // I limit
		o_limit); // O limit
	reset_pid(pid);
	return offset;
}

float do_pid(pid_t *pid, float p_error, float d_error)
{
	float p_result = p_error * pid->p_gain;
	float d_result = d_error * pid->d_gain;

	pid->error_accum += p_error;
	pid->counter++;
	if(pid->counter >= truck.pid_downsample)
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




void handle_controls()
{
	
	DISABLE_INTERRUPTS


	ENABLE_INTERRUPTS
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

void handle_radio()
{
	if(radio.packet[0] != SYNC_CODE) return;

	uint16_t chksum = get_chksum(radio.packet, PACKET_SIZE - 2);

	if((chksum & 0xff) == radio.packet[PACKET_SIZE - 2] &&
		((chksum >> 8) & 0xff) == radio.packet[PACKET_SIZE - 1])
	{
// packet good
		radio_led_toggle();

//TRACE2
//print_hex2(radio.packet[2]);
		DISABLE_INTERRUPTS

		truck.radio_timeout = TIMEOUT_RELOAD;
		truck.throttle_reverse = BIT_IS_CLEAR(radio.packet[2], 0) ? 0 : 1;
		truck.throttle = BIT_IS_CLEAR(radio.packet[2], 1) ? THROTTLE_MAX : 0;
		truck.steering = STEERING_MID;

// low speed steering
		if(BIT_IS_CLEAR(radio.packet[2], 3)) truck.steering = 2;
		else
		if(BIT_IS_CLEAR(radio.packet[2], 4)) truck.steering = 3;
		else
// high speed steering
		if(BIT_IS_CLEAR(radio.packet[2], 2)) truck.steering = 1;
		else
		if(BIT_IS_CLEAR(radio.packet[2], 5)) truck.steering = 4;

		ENABLE_INTERRUPTS

if(truck.steering)
{
TRACE
print_number(truck.steering);
}
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
			begin_gyro_calibration();
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
}

void dump_config()
{
	TRACE2
	print_text("\nheadlights_on=");
	print_number(truck.headlights_on);
	print_text("\nmid_steering=");
	print_number(truck.mid_steering);
	print_text("\nmid_throttle=");
	print_number(truck.mid_throttle);
	print_text("\nmax_throttle_fwd=");
	print_number(truck.max_throttle_fwd);
	print_text("\nmax_throttle_rev=");
	print_number(truck.max_throttle_rev);
	print_text("\nthrottle_base=");
	print_number(truck.throttle_base);
	print_text("\nmax_steering=");
	print_number(truck.max_steering);
	print_text("\nmin_steering=");
	print_number(truck.min_steering);
	print_text("\nauto_steering=");
	print_number(truck.auto_steering);
	print_text("\nauto_throttle=");
	print_number(truck.auto_throttle);
	print_text("\nrpm_dv_size=");
	print_float(truck.rpm_dv_size);
	print_text("\ngyro_center_max=");
	print_number(truck.gyro_center_max);
	print_text("\nmax_gyro_drift=");
	print_float(truck.max_gyro_drift);
	print_text("\nangle_to_gyro=");
	print_number(truck.angle_to_gyro);
	print_text("\nthrottle_ramp_delay=");
	print_number(truck.throttle_ramp_delay);
	print_text("\nthrottle_ramp_step=");
	print_number(truck.throttle_ramp_step);
	print_text("\npid_downsample=");
	print_number(truck.pid_downsample);
	print_text("\nsteering_step_delay=");
	print_number(truck.steering_step_delay);
	print_text("\nbattery_analog=");
	print_number(truck.battery_analog);
	print_text("\ntarget_rpm=");
	print_float(truck.target_rpm);
	print_text("\ntarget_power=");
	print_float(truck.target_power);

	print_text("\npath_x=");
	print_number(truck.path_x);
	print_text("\npath_feedback=");
	print_float(truck.path_feedback);

	print_text("\nbattery_v0=");
	print_float(truck.battery_v0);
	print_text("\nsteering_step=");
	print_float(TO_DEG(truck.steering_step));
	print_text("\nsteering_overshoot=");
	print_float(TO_DEG(truck.steering_overshoot));
	print_text("\npower_base=");
	print_float(truck.power_base);
	print_text("\nrpm_slope=");
	print_float(truck.rpm_slope);

	print_text("\nsteering PID=");
	print_float(truck.heading_pid.p_gain);
	print_float(truck.heading_pid.i_gain);
	print_float(truck.heading_pid.d_gain);
	print_float(truck.heading_pid.i_limit);
	print_float(truck.heading_pid.o_limit);
	print_lf();

	print_text("throttle PID=");
	print_float(truck.throttle_pid.p_gain);
	print_float(truck.throttle_pid.i_gain);
	print_float(truck.throttle_pid.d_gain);
	print_float(truck.throttle_pid.i_limit);
	print_float(truck.throttle_pid.o_limit);
	print_lf();

	print_text("RPM PID=");
	print_float(truck.rpm_pid.p_gain);
	print_float(truck.rpm_pid.i_gain);
	print_float(truck.rpm_pid.d_gain);
	print_float(truck.rpm_pid.i_limit);
	print_float(truck.rpm_pid.o_limit);
	print_lf();

}

/*
 * void update_headlights()
 * {
 * 	if(truck.headlights_on)
 * 	{
 * 		SET_PIN(HEADLIGHT_GPIO, HEADLIGHT_PIN);
 * 	}
 * 	else
 * 	{
 * 		CLEAR_PIN(HEADLIGHT_GPIO, HEADLIGHT_PIN);
 * 	}
 * }
 * 
 */

int read_config_packet(const unsigned char *buffer)
{
	int offset = 0;
	truck.headlights_on = buffer[offset++];
	truck.mid_steering = buffer[offset++];
	truck.mid_throttle = buffer[offset++];
	truck.max_throttle_fwd = buffer[offset++];
	truck.max_throttle_rev = buffer[offset++];
	truck.throttle_base = buffer[offset++];
	truck.max_steering = buffer[offset++];
	truck.min_steering = buffer[offset++];
	truck.auto_steering = buffer[offset++];
	truck.auto_throttle = buffer[offset++];
	truck.rpm_dv_size = buffer[offset++];

	truck.gyro_center_max = READ_UINT16(buffer, offset);
	truck.max_gyro_drift = (float)READ_UINT16(buffer, offset) / 256;
	truck.angle_to_gyro = READ_UINT16(buffer, offset);
	truck.throttle_ramp_delay = READ_UINT16(buffer, offset);
	truck.throttle_ramp_step = READ_UINT16(buffer, offset);
	truck.pid_downsample = READ_UINT16(buffer, offset);
	truck.steering_step_delay = READ_UINT16(buffer, offset);
	truck.battery_analog = READ_UINT16(buffer, offset);
	truck.target_rpm = READ_UINT16(buffer, offset);
	truck.path_x = READ_UINT16(buffer, offset);
	
	
	truck.path_feedback = READ_FLOAT32(buffer, offset);
	truck.target_power = READ_FLOAT32(buffer, offset);
	truck.battery_v0 = READ_FLOAT32(buffer, offset);
	truck.steering_step = READ_FLOAT32(buffer, offset);
	truck.steering_overshoot = READ_FLOAT32(buffer, offset);

	truck.power_base = READ_FLOAT32(buffer, offset);
	truck.rpm_slope = READ_FLOAT32(buffer, offset);

	offset = read_pid(&truck.heading_pid, buffer, offset);
	offset = read_pid(&truck.throttle_pid, buffer, offset);
	offset = read_pid(&truck.rpm_pid, buffer, offset);

	resize_derivative(&truck.rpm_dv, truck.rpm_dv_size);

TRACE
print_text("offset=");
print_number(offset);


//	update_headlights();

// debug
//truck.max_throttle_fwd = 0;
//truck.max_throttle_rev = 0;
//truck.auto_steering = 1;

	return offset;
}

void save_config(unsigned char *buffer, int bytes)
{
	const unsigned char buffer2[] = 
	{
		(SETTINGS_MAGIC >> 24) & 0xff,
		(SETTINGS_MAGIC >> 16) & 0xff,
		(SETTINGS_MAGIC >> 8) & 0xff,
		SETTINGS_MAGIC & 0xff,
	};

   	USART_Cmd(USART3, DISABLE);
   	USART_Cmd(USART1, DISABLE);

	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
                	FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
    if (FLASH_EraseSector(FLASH_Sector_3, VoltageRange_3) != FLASH_COMPLETE)
	{
	}

	/* Clear pending flags (if any) */  
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | 
        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR|FLASH_FLAG_PGSERR); 
	int address = SETTINGS_ADDRESS;
	if(FLASH_ProgramWord(address, *(int*)(buffer2)) != FLASH_COMPLETE)
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
	TRACE2
	print_text("Saved flash config\n");
}

void load_config()
{
// Load settings from flash
	const unsigned char *buffer = (unsigned char*)SETTINGS_ADDRESS;
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

void handle_beacon()
{
	unsigned char *receive_buf = truck.bluetooth.receive_buf;
	int receive_size = truck.bluetooth.receive_size;
	uint16_t chksum = get_chksum(receive_buf, 
		receive_size - 2);

	if((chksum & 0xff) == receive_buf[receive_size - 2] &&
		((chksum >> 8) & 0xff) == receive_buf[receive_size - 1])
	{

		switch(receive_buf[6])
		{
// battery voltage
			case 0:
			{
// get controls
				if(receive_buf[8])
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
						begin_gyro_calibration();
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
				{
					DISABLE_INTERRUPTS
					truck.have_bt_controls = 0;
					ENABLE_INTERRUPTS
				}

// create return packet
				int offset = 0;
				truck.bluetooth.send_buf[offset++] = 0xff;
				truck.bluetooth.send_buf[offset++] = 0x2d;
				truck.bluetooth.send_buf[offset++] = 0xd4;
				truck.bluetooth.send_buf[offset++] = 0xe5;
// size
				WRITE_INT16(truck.bluetooth.send_buf, offset, 30);
// battery response
				truck.bluetooth.send_buf[offset++] = 0;
				truck.bluetooth.send_buf[offset++] = 0;
				
				WRITE_INT32(truck.bluetooth.send_buf, offset, truck.battery);
				WRITE_FLOAT32(truck.bluetooth.send_buf, offset, truck.battery_voltage);
				WRITE_INT16(truck.bluetooth.send_buf, offset, (int)truck.gyro_center);
				WRITE_INT16(truck.bluetooth.send_buf, offset, truck.gyro_max - truck.gyro_min);
				WRITE_INT16(truck.bluetooth.send_buf, offset, truck.rpm);
				WRITE_FLOAT32(truck.bluetooth.send_buf, offset, truck.current_heading);
//				WRITE_FLOAT32(truck.bluetooth.send_buf, offset, truck.power);
				WRITE_INT16(truck.bluetooth.send_buf, offset, truck.path_x);

				chksum = get_chksum(truck.bluetooth.send_buf, offset);
				WRITE_INT16(truck.bluetooth.send_buf, offset, chksum);
				truck.bluetooth.send_offset = 0;
				truck.bluetooth.send_size = offset;
				
//	TRACE2
//	print_buffer(truck.bluetooth.send_buf, truck.bluetooth.send_size);
				break;
			}

// reset gyro
			case 1:
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
			case 2:
			{
				int offset = 8;
TRACE2
print_buffer(truck.bluetooth.receive_buf, truck.bluetooth.receive_size);

				int bytes = read_config_packet(receive_buf + offset);
				int need_save = receive_buf[7];

				dump_config();

				if(need_save)
				{
					save_config(receive_buf + offset,
						bytes);

					CLEAR_PIN(LED_GPIO1, LED_PIN1);
					SET_PIN(LED_GPIO2, LED_PIN2);
					int i;
					int prev_timer_h = truck.timer_high;
					int toggle = 0;
					int mid_steering_pwm = MIN_PWM + 
						(MAX_PWM - MIN_PWM) * 
						truck.mid_steering /
						100;
					int steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
						truck.min_steering / 
						100;
					truck.writing_settings = 1;

					for(i = 0; i < 2; i++)
					{
						if(toggle == 0)
						{
							truck.steering_pwm = mid_steering_pwm - steering_magnitude;
							write_pwm();
						}
						else
						{
							truck.steering_pwm = mid_steering_pwm + steering_magnitude;
							write_pwm();
						}
						toggle ^= 1;

						while(1)
						{
							DISABLE_INTERRUPTS
							int time_difference = truck.timer_high - prev_timer_h;
							ENABLE_INTERRUPTS
	// seems required because of a compiler error
							flush_uart();
							if(time_difference >= TIMER_HZ / 2) break;
						}

						DISABLE_INTERRUPTS
						prev_timer_h = truck.timer_high;
						ENABLE_INTERRUPTS

						TOGGLE_PIN(LED_GPIO1, LED_PIN1);
						TOGGLE_PIN(LED_GPIO2, LED_PIN2);
					}

					truck.writing_settings = 0;
					CLEAR_PIN(LED_GPIO1, LED_PIN1);
					SET_PIN(LED_GPIO2, LED_PIN2);
				}
				break;
			}
		}
	}
}

void get_code1();

void get_data()
{
	truck.bluetooth.receive_buf[truck.bluetooth.receive_offset++] = 
		truck.bluetooth.data;
	if(truck.bluetooth.receive_offset >= truck.bluetooth.receive_size)
	{
		truck.bluetooth.got_data = 1;
		truck.bluetooth.current_function = get_code1;
	}
}

void get_size()
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

void get_code4()
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
		truck.bluetooth.current_function = get_code1;
}


void get_code3()
{
	if(truck.bluetooth.data == 0xd4)
		truck.bluetooth.current_function = get_code4;
	else
		truck.bluetooth.current_function = get_code1;
}


void get_code2()
{
	if(truck.bluetooth.data == 0x2d)
		truck.bluetooth.current_function = get_code3;
	else
		truck.bluetooth.current_function = get_code1;
}


void get_code1()
{
	if(truck.bluetooth.data == 0xff)
		truck.bluetooth.current_function = get_code2;
}


void USART3_IRQHandler(void)
{
	truck.bluetooth.data = USART3->DR;
	truck.bluetooth.current_function();
}

// TIM10 wraps at TIMER_HZ
void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
		
		truck.timer_high++;

// Update shutdown timer
		if(truck.bt_timeout > 0)
		{
			truck.bt_timeout--;
		}
		if(truck.radio_timeout > 0)
		{
			truck.radio_timeout--;
		}
	}
}


void handle_bluetooth()
{

#ifdef BLUETOOTH_PASSTHROUGH
	if(uart_got_input() && (USART3->SR & USART_FLAG_TC) != 0)
	{
		unsigned char c = uart_get_input();

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
	USART_InitStructure.USART_BaudRate = 115200;
// set to configure a new device
//	USART_InitStructure.USART_BaudRate = 9600;
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

// Gyro
	if((ADC2->SR & ADC_FLAG_EOC))
	{
		truck.gyro_accum += ADC2->DR;
		ADC_SoftwareStartConv(ADC2);
		truck.gyro_count++;
		if(truck.gyro_count >= GYRO_OVERSAMPLE)
		{
			DISABLE_INTERRUPTS
			truck.gyro = (float)truck.gyro_accum / truck.gyro_count -
				truck.ref;
			ENABLE_INTERRUPTS
			
			truck.gyro_accum = 0;
			truck.gyro_count = 0;
TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);





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
				print_number(truck.gyro_center_accum);
				print_number(truck.gyro_center_count);
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
				truck.current_heading += (truck.gyro - truck.gyro_center) / 
					truck.angle_to_gyro /
					NAV_HZ;
				truck.current_heading = fix_angle(truck.current_heading);
				ENABLE_INTERRUPTS
				
//				truck.debug_counter++;
//				if(truck.debug_counter >= 128)
//				{
//					truck.debug_counter = 0;
//					TRACE2
//					print_float(TO_DEG(truck.current_heading));
//				}
			}
		}
	}


	if((ADC3->SR & ADC_FLAG_EOC))
	{
		if(truck.sample_ref)
		{
			truck.ref_accum += ADC3->DR;

			ADC_RegularChannelConfig(ADC3, 
				ADC_Channel_13, 
				1, 
				ADC_SampleTime_480Cycles);
			ADC_SoftwareStartConv(ADC3);

			truck.sample_ref = 0;
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
		else
		{
			truck.current_accum += ADC3->DR;
			ADC_RegularChannelConfig(ADC3, 
				ADC_Channel_2, 
				1, 
				ADC_SampleTime_480Cycles);
			ADC_SoftwareStartConv(ADC3);
			truck.sample_ref = 1;
			truck.current_count++;

			if(truck.current_count >= CURRENT_OVERSAMPLE)
			{
				truck.raw_current = (float)truck.current_accum / truck.current_count;

				DISABLE_INTERRUPTS
//				truck.current = (truck.raw_current - CURRENT_BASE) / CURRENT_SCALE;
// battery_voltage is only a few hundreths off
//				truck.power = truck.current * truck.battery_voltage;
// Not connected
				truck.current = 0;
				truck.power = 0;
				ENABLE_INTERRUPTS

				truck.current_accum = 0;
				truck.current_count = 0;

//				TRACE2
//				print_float(truck.current);
//				print_float(truck.battery_voltage);
//				print_float(truck.power);
//				print_number(truck.target_rpm2);
//				print_float(truck.throttle_feedback);
			}
		}
	}
	
}


void init_analog()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
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
		GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
// must call this before ENABLE
	ADC_RegularChannelConfig(ADC1, 
		ADC_Channel_0, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC2, 
		ADC_Channel_1, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC3, 
		ADC_Channel_2, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_Cmd(ADC3, ENABLE);
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);
	ADC_SoftwareStartConv(ADC3);
	
	truck.sample_ref = 1;

}

void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_FLAG_Update)
	{
		TIM2->SR = ~TIM_FLAG_Update;
//		SET_PIN(GPIOA, GPIO_Pin_6);
		SET_PIN(GPIOC, GPIO_Pin_4);

		if(truck.writing_settings) return;

		int mid_steering_pwm = MIN_PWM + 
			(MAX_PWM - MIN_PWM) * 
			truck.mid_steering /
			100;
		int mid_throttle_pwm = MIN_PWM + 
			(MAX_PWM - MIN_PWM) * 
			truck.mid_throttle / 
			100;
		int max_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.max_steering / 
			100;
		int min_steering_magnitude = (MAX_PWM - MIN_PWM) / 2 * 
			truck.min_steering / 
			100;
		int throttle_ramp_step = (MAX_PWM - MIN_PWM) *
			truck.throttle_ramp_step / 
			100;


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
#ifdef DO_THROTTLE_TEST
				truck.target_power = THROTTLE_POWER1;
				truck.throttle_time = truck.timer_high;
#endif
				truck.throttle_state = THROTTLE_RAMP;
				truck.throttle_pwm = mid_throttle_pwm;
				truck.current_heading = 0;
				truck.throttle_ramp_counter = 0;
				reset_pid(&truck.heading_pid);
				reset_pid(&truck.throttle_pid);
				reset_pid(&truck.rpm_pid);
			}

// change in steering
			if(truck.steering != truck.steering2)
			{
				truck.throttle_ramp_counter = 0;
				truck.steering_step_counter = 0;
			}
			
			truck.steering2 = truck.steering;
			truck.throttle2 = truck.throttle;
			truck.throttle_reverse2 = truck.throttle_reverse;


			int need_feedback = 1;
			int throttle_magnitude = 0;
			if(truck.throttle_reverse ||
				(truck.have_bt_controls && truck.bt_throttle < 0))
			{
				throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
					truck.max_throttle_rev / 
					100;
// don't use steering feedback in reverse
				need_feedback = 0;
			}
			else
			{
				throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
					truck.max_throttle_fwd / 
					100;
			}

			if(truck.have_bt_controls)
			{
				need_feedback = 0;
				int target_throttle = mid_throttle_pwm -
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
						if(truck.throttle_ramp_counter >= truck.throttle_ramp_delay)
						{
							truck.throttle_ramp_counter = 0;
							truck.throttle_pwm -= throttle_ramp_step;
							truck.throttle_pwm = MAX(truck.throttle_pwm, 
								mid_throttle_pwm -
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
						if(truck.throttle_ramp_counter >= truck.throttle_ramp_delay)
						{
							truck.throttle_ramp_counter = 0;
							truck.throttle_pwm += throttle_ramp_step;
							truck.throttle_pwm = MIN(truck.throttle_pwm, 
								mid_throttle_pwm +
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
					truck.throttle_pwm = mid_throttle_pwm;
				}
				
				truck.steering_pwm = mid_steering_pwm +
					max_steering_magnitude *
					truck.bt_steering / 
					127;
			}
			else
// stick controller
			if(truck.throttle > 0)
			{
#ifdef DO_THROTTLE_TEST
				need_feedback = 0;
				if(truck.timer_high - truck.throttle_time >= 
					THROTTLE_DELAY * TIMER_HZ)
				{
					if(truck.rpm < THROTTLE_RPM2)
					{
						truck.target_power += POWER_STEP;
					}

					truck.throttle_time = truck.timer_high;
				}
#endif

				if(truck.throttle_reverse ||
					truck.throttle_state == THROTTLE_RAMP)
				{
					truck.throttle_ramp_counter++;
					if(truck.throttle_ramp_counter >= truck.throttle_ramp_delay)
					{
						truck.throttle_ramp_counter = 0;
						
						if(truck.throttle_reverse)
						{
// reverse ramp based on PWM target
							truck.throttle_pwm += throttle_ramp_step;

							truck.throttle_pwm = MIN(mid_throttle_pwm +
								throttle_magnitude,
								truck.throttle_pwm);
						}
						else
						{
							if(truck.auto_throttle)
							{
								throttle_magnitude = (MAX_PWM - MIN_PWM) / 2 *
									truck.throttle_base / 
									100;
							}

							if(truck.throttle_pwm > mid_throttle_pwm -
									throttle_magnitude)
							{
								truck.throttle_pwm -= throttle_ramp_step;
							}
							else
							{
								truck.throttle_state = THROTTLE_AUTO;
								truck.start_time = truck.timer_high;
#ifdef DO_RPM_TEST
								truck.target_rpm = RPM1;
#endif
							}
						}
					}

/*
 * 		TRACE2
 * 		print_text("throttle_pwm=");
 * 		print_number(truck.throttle_pwm);
 */

				}
				else
				{
#ifdef POWER_FEEDBACK
// forward based on power feedback
// scale target power based on voltage & target power
					truck.throttle_feedback = do_pid(&truck.throttle_pid,
						truck.target_power - truck.power,
						0);

					truck.throttle_pwm = mid_throttle_pwm - 
						(MAX_PWM - MIN_PWM) / 2 *
						(truck.throttle_base + truck.throttle_feedback) /
						100;
#else // POWER_FEEDBACK
					if(truck.auto_throttle)
					{


#ifdef DO_RPM_TEST
						truck.target_rpm = RPM1 + 
							(RPM2 - RPM1) *
							(truck.timer_high - truck.start_time) /
							(60 * TIMER_HZ);
#endif
						truck.target_rpm2 = truck.target_rpm;
						if(truck.power > truck.power_base)
						{
							truck.target_rpm2 += (truck.power - truck.power_base) *
								truck.rpm_slope;
							truck.target_rpm2 = MAX(0, truck.target_rpm2);
						}

						truck.throttle_feedback = do_pid(&truck.rpm_pid,
							(float)(truck.target_rpm2 - truck.rpm) / 1000,
							-get_derivative(&truck.rpm_dv) / 1000);
						truck.throttle_pwm = mid_throttle_pwm - 
							(MAX_PWM - MIN_PWM) / 2 *
							(truck.throttle_base + truck.throttle_feedback) /
							100;
#ifdef DO_PWM_TEST
						truck.throttle_pwm = mid_throttle_pwm - 
							(MAX_PWM - MIN_PWM) / 2 *
							(PWM2 - PWM1) *
							(truck.timer_high - truck.start_time) /
							TEST_TIME / 
							100;
#endif
					}
					else
					{
						truck.throttle_pwm = mid_throttle_pwm - 
							(MAX_PWM - MIN_PWM) / 2 *
							truck.max_throttle_fwd /
							100;
					}
#endif // !POWER_FEEDBACK
				}

// steering with throttle
				float steering_overshoot = 0;
				switch(truck.steering)
				{
// full left
					case 1:
						truck.steering_pwm = mid_steering_pwm - max_steering_magnitude;
						steering_overshoot = truck.steering_overshoot;
						need_feedback = 0;
						break;
// slow left
					case 2:
						if(!truck.auto_steering ||
							truck.steering_first ||
							truck.throttle_reverse)
						{
							need_feedback = 0;
							truck.steering_pwm = mid_steering_pwm - min_steering_magnitude;
						}
						else
						{
							truck.steering_step_counter++;
							if(truck.steering_step_counter >= truck.steering_step_delay)
							{
								truck.steering_step_counter = 0;
								truck.current_heading += truck.steering_step;
//TRACE2
print_float(TO_DEG(truck.current_heading));
							}
						}
						break;
// slow right
					case 3:
						if(!truck.auto_steering ||
							truck.steering_first ||
							truck.throttle_reverse)
						{
							need_feedback = 0;
							truck.steering_pwm = mid_steering_pwm + min_steering_magnitude;
						}
						else
						{
							truck.steering_step_counter++;
							if(truck.steering_step_counter >= truck.steering_step_delay)
							{
								truck.steering_step_counter = 0;
								truck.current_heading -= truck.steering_step;
//TRACE2
print_float(TO_DEG(truck.current_heading));
							}
						}
						break;
// full right
					case 4:
						truck.steering_pwm = mid_steering_pwm + max_steering_magnitude;
						steering_overshoot = -truck.steering_overshoot;
						need_feedback = 0;
						break;
					
					default:
// Force next steering press to use heading hold
						truck.steering_first = 0;

						if(!truck.auto_steering)
						{
							need_feedback = 0;
							truck.steering_pwm = mid_steering_pwm;
						}
						else
						{
// path following
							truck.steering_step_counter++;
							if(truck.steering_step_counter >= truck.steering_step_delay)
							{
								truck.steering_step_counter = 0;
								if(truck.path_feedback != 0)
								{
									truck.current_heading += TO_RAD(truck.path_feedback);
								}
//TRACE2
print_float(TO_DEG(truck.current_heading));
							}
						}
						break;
				}
				

				if(need_feedback)
				{
// -100 - 100
					float steering_feedback = do_pid(&truck.heading_pid, 
						truck.current_heading, 
						(float)(truck.gyro - truck.gyro_center) /
							truck.angle_to_gyro);
					truck.steering_pwm = mid_steering_pwm -
						steering_feedback * 
						(MAX_PWM - MIN_PWM) / 2 / 
						100;
				}
				else
				{
//					reset_pid(&truck.throttle_pid);
//					reset_pid(&truck.rpm_pid);
					reset_pid(&truck.heading_pid);
					truck.current_heading = steering_overshoot;
				}

/*
 * 				TRACE2
 * 				print_float(steering_feedback);
 */


				
			}
			else
// steering with no throttle
			{
				truck.throttle_pwm = mid_throttle_pwm;
				switch(truck.steering)
				{
// full left
					case 1:
						truck.steering_pwm = mid_steering_pwm - max_steering_magnitude;
						truck.steering_first = 1;
						break;
// slow left
					case 2:
						truck.steering_pwm = mid_steering_pwm - min_steering_magnitude;
						truck.steering_first = 1;
						break;
// slow right
					case 3:
						truck.steering_pwm = mid_steering_pwm + min_steering_magnitude;
						truck.steering_first = 1;
						break;
// full right
					case 4:
						truck.steering_pwm = mid_steering_pwm + max_steering_magnitude;
						truck.steering_first = 1;
						break;
// no steering
					default:
						truck.steering_pwm = mid_steering_pwm;
						truck.steering_first = 0;
						break;
				}

			}


			write_pwm();
		}
		else
		{
			truck.throttle_pwm = mid_throttle_pwm;
			truck.steering_pwm = mid_steering_pwm;
			
			write_pwm();
		}
	}

	if(TIM2->SR & TIM_FLAG_CC1)
	{
		TIM2->SR = ~TIM_FLAG_CC1;
//		CLEAR_PIN(GPIOA, GPIO_Pin_6);
	}

	if(TIM2->SR & TIM_FLAG_CC2)
	{
		TIM2->SR = ~TIM_FLAG_CC2;
		CLEAR_PIN(GPIOC, GPIO_Pin_4);
	}
}

void write_pwm()
{
	CLAMP(truck.steering_pwm, MIN_PWM, MAX_PWM);
	CLAMP(truck.throttle_pwm, MIN_PWM, MAX_PWM);
	SET_COMPARE(TIM2, CCR1, truck.throttle_pwm);
	SET_COMPARE(TIM2, CCR2, truck.steering_pwm);
	
	int mid_throttle_pwm = MIN_PWM + 
		(MAX_PWM - MIN_PWM) * 
		truck.mid_throttle / 
		100;
	if(truck.throttle_pwm > mid_throttle_pwm)
	{
		int pwm = (truck.throttle_pwm - mid_throttle_pwm) * 
			MAX_PERIOD /
			((MAX_PWM - MIN_PWM) / 2);
// strange artifact makes it die if below a certain amount
		pwm = MAX(pwm, 4000);
		SET_COMPARE(TIM3, CCR2, MAX_PERIOD - pwm);
		SET_COMPARE(TIM3, CCR1, MAX_PERIOD);
	}
	else
	if(truck.throttle_pwm < mid_throttle_pwm)
	{
		int pwm = (mid_throttle_pwm - truck.throttle_pwm) * 
			MAX_PERIOD /
			((MAX_PWM - MIN_PWM) / 2);
		pwm = MAX(pwm, 4000);
		SET_COMPARE(TIM3, CCR2, MAX_PERIOD);
		SET_COMPARE(TIM3, CCR1, MAX_PERIOD - pwm);
	}
	else
	{
		SET_COMPARE(TIM3, CCR2, MAX_PERIOD);
		SET_COMPARE(TIM3, CCR1, MAX_PERIOD);
	}
	
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


//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);


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

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
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



 	NVIC_InitTypeDef NVIC_InitStructure;
 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
 	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
 	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(TIM2, 
		TIM_IT_Update | 
			TIM_IT_CC1 |
			TIM_IT_CC2, 
		ENABLE);
	
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
	
	DISABLE_INTERRUPTS
	if(truck.timer_high - truck.rpm_time >= TIMER_HZ / 10)
	{
		truck.rpm_time = truck.timer_high;
		truck.rpm = truck.rpm_counter * 300 / 16;
		truck.rpm_counter = 0;
		ENABLE_INTERRUPTS

#ifdef DO_RPM_TEST
		TRACE
		print_number(truck.target_rpm);
		print_number(truck.rpm);
		if(truck.rpm > 10)
			print_float((float)60 * 1609363 / (float)(truck.rpm * 110 * 3.141 * 60));
		else
			print_float(0);
#endif

#ifdef DO_PWM_TEST
		TRACE
		print_number(TIM3->CCR1);
		print_number(TIM3->CCR2);
		print_number(truck.throttle_pwm);
		print_number(truck.rpm);
#endif

		update_derivative(&truck.rpm_dv, truck.rpm);

//		if(truck.throttle > 0)
		{
/*
 * 			TRACE2
 * 			print_text("rpm=");
 * 			print_number(truck.rpm);
 * 			print_text("pwm=");
 * 			print_number(truck.throttle_pwm);
 */

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

void init_spi()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
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
			if(truck.spi_counter >= 16)
			{
				truck.have_spi = 1;
				truck.spi_state = SPI_SYNC_CODE;
			}
			break;
	}
  }
}

void handle_spi()
{
	if(truck.have_spi)
	{
		truck.path_x = truck.spi_buffer;
		truck.have_spi = 0;
		truck.spi_buffer = 0;
		TRACE
		print_number(truck.path_x);
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

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();
	
	truck.steering = STEERING_MID;


	truck.headlights_on = 0;
	truck.mid_steering = 50;
	truck.mid_throttle = 50;
	truck.max_throttle_fwd = 100;
	truck.max_throttle_rev = 100;
	truck.max_steering = 100;
	truck.min_steering = 25;
	truck.auto_steering = 1;
	truck.auto_throttle = 1;
	
	truck.gyro_center_max = 100;
	truck.max_gyro_drift = 0.1f;
	truck.angle_to_gyro = 450;
	truck.throttle_ramp_delay = 0;
	truck.throttle_ramp_step = 1;
	truck.target_power = 15.0f;
	truck.power_base = 20.0f;
	truck.rpm_slope = 0;
	truck.target_rpm = 500;
	truck.pid_downsample = 1;
	truck.steering_step_delay = 0;
	truck.steering_step = TO_RAD(30) / PWM_HZ;
	truck.steering_overshoot = 0;
	truck.battery_analog = 965;
	truck.battery_v0 = 8.67f;

// heading error -> PWM
	init_pid(&truck.heading_pid, 
		30, // P gain
		0.5f, // I gain	
		10, // D gain
		100, // I limit
		100); // O limit

// power error -> throttle
	init_pid(&truck.throttle_pid, 
		0, // P gain
		1.0f, // I gain	
		0, // D gain
		100, // I limit
		100); // O limit

// RPM error -> throttle
	init_pid(&truck.rpm_pid, 
		0, // P gain
		0, // I gain	
		0, // D gain
		100, // I limit
		100); // O limit

	truck.rpm_dv_size = 10;
	init_derivative(&truck.rpm_dv, truck.rpm_dv_size);

	load_config();
	dump_config();


	truck.throttle_pwm = MIN_PWM + (MAX_PWM - MIN_PWM) * truck.mid_throttle / 100;
	truck.steering_pwm = MIN_PWM + (MAX_PWM - MIN_PWM) * truck.mid_steering / 100;


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
	
	TIM_ITConfig(TIM10, 
		TIM_IT_Update, 
		ENABLE);

	print_text("Welcome to truck\n");
	flush_uart();

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
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
	GPIO_InitStructure.GPIO_Pin = RPM_PIN;
	GPIO_Init(RPM_GPIO, &GPIO_InitStructure);
	truck.rpm_time = truck.timer_high;
	truck.rpm_status = PIN_IS_SET(RPM_GPIO, RPM_PIN);





//	update_headlights();

	init_bluetooth();
	init_analog();
	init_pwm();
	init_spi();

	init_cc1101();
	cc1101_receiver();

// DEBUG
// bypass calibration for testing
//	imu.have_gyro_center = 1;

//	test_motors();
	
// test floating point
//	float x = 0.12345f;
//	float y = tanf(x);
//    print_float(x);
//    print_float(y);
	

	while(1)
	{

		handle_uart();
		handle_bluetooth();
		handle_analog();
		handle_rpm();
		handle_spi();
		
		if(radio.got_packet)
		{
			radio.got_packet = 0;
			handle_radio();
		}





	}
	
		
}





