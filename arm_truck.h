/*
 * STM32 Controller for converted lunchbox
 * Copyright (C) 2012-2020 Adam Williams <broadcast at earthling dot net>
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



#ifndef ARM_TRUCK_H
#define ARM_TRUCK_H

#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "arm_math.h"
#include "arm_nav.h"
#include "arm_imu.h"


// select the vehicle
//#define IS_CAR



// all vehicles use brushless motors
#define USE_BRUSHLESS


// external navigation using a raspberry pi
//#define SPI_NAV
// external navigation using an Odroid
//#define UART_NAV
// pass bluetooth to debug port
//#define BLUETOOTH_PASSTHROUGH
// before a device is configured, its UART will be 9600
//#define BLUETOOTH9600 

#ifndef IS_CAR

// truck uses MPU6000.  Car uses analog gyro.  Comment this out if car.
	#define I2C_IMU
// truck's ESC pin
	#define ESC_PIN GPIO_Pin_6
// reverse steering servo on truck
//	#define REVERSE_STEERING

// use the xbee 900
    #define USE_XBEE
// use frequency hopping on the xbee.  Doesn't work
//  #define USE_HOPPING

#else  // !IS_CAR
// truck's ESC pin burned out.  Use a different pin in the same port.
	#define ESC_PIN GPIO_Pin_5

    #define REVERSE_GYRO

// use the CC1101
    #define USE_CC1101

#endif // IS_CAR







// gyro update rate
#ifndef I2C_IMU
#define NAV_HZ 320
#else // !I2C_IMU
//#define NAV_HZ 1100
#define NAV_HZ 366
#endif // I2C_IMU

// tick/TIM10 wraps at this frequency
#define TIMER_HZ 100

// how fast the feedback loop fires
#define PWM_HZ 100
// what the feedback is calibrated for
#define PWM_BASE 50


#define PACKET_SIZE 8

// timer for packet flashing
#define LED_DELAY2 2
// green pin when high
#define LED_PIN1 GPIO_Pin_1
#define LED_GPIO1 GPIOB
// red pin when high
#define LED_PIN2 GPIO_Pin_2
#define LED_GPIO2 GPIOB

typedef struct
{
	float p_gain;
	float i_gain;
	float d_gain;
	float d2_gain;

	float p_limit;
	float i_limit;
	float d_limit;
	int counter;
	float error_accum;

	float p_input;
	float d_input;
	float d2_input;

	float p_result;
	float i_result;
	float d_result;
	float d2_result;
// the sign of the current error
//	int stiction_sign;

	int ignore_i; // don't wind up if over the output limit
// amount of change before engaging stiction_amount
//	float stiction_threshold;
//	float stiction_amount;
//	float max_result;

// last result for debugging
	float result;
} pid_t;


#define BLUE_BUFSIZE 256
typedef struct
{
	void (*current_function)();
	
	unsigned char receive_buf[BLUE_BUFSIZE];
	unsigned char receive_buf2[BLUE_BUFSIZE];
	unsigned char send_buf[BLUE_BUFSIZE];
	int receive_offset;
	int receive_size;
	int send_offset;
	int send_size;
	int counter;
	unsigned char data;
	int got_data;
} bluetooth_t;


#define ORDER 2
typedef struct
{
	float bandwidth;
	float prev_output[ORDER];
	float prev_input[ORDER];
	float result;
} filter_t;



typedef struct
{
// the mane timer which increments at TIMER_HZ
	int tick;
// a time used by the throttle
	int start_time;
// data from radio	
	int throttle_reverse;
	int throttle_reverse2;
// 0 - not driving
	int throttle;
	int throttle2;
	int steering;
	int steering2;
	int bt_throttle;
	int bt_steering;
	int have_bt_controls;

// PWM values
	int throttle_pwm;
	int steering_pwm;
	int led_counter;

// starting PWM for THROTTLE_RAMP state & minimum throttle setting
	int min_throttle_fwd100;
	int min_throttle_rev100;

// PWM for THROTTLE_WAIT state
	int throttle_base;
	int throttle_reverse_base;

// maximum throttle setting
	int max_throttle_fwd100;
	int max_throttle_rev100;

	int throttle_state;
#define THROTTLE_OFF 0 
#define THROTTLE_RAMP 1
#define THROTTLE_WAIT 2
#define THROTTLE_AUTO 3

// number of PWM cycles for each throttle step
	int throttle_ramp_delay;
	int throttle_ramp_counter;
	int throttle_ramp_step100;



	pid_t heading_pid;
	filter_t p_filter;
	filter_t d_filter;
	filter_t d2_filter;
// 0 - 100
	float d_bandwidth;
	
	pid_t rpm_pid;
// vanishing point feedback
// P=1/16deg
//	pid_t path_pid;
// side of path feedback
// P=1/16deg
//	pid_t side_pid;
	imu_t imu;

	int battery;
	int battery_accum;
	int battery_count;
// voltage in V
	float battery_voltage;
// ADC value of battery voltage
	int battery_analog;
	float battery_v0;

// current steering command was pressed before throttle so use fixed PWM
	int steering_first;
// Make the interrupt handler not copy the radio to the steering PWM
	int writing_settings;
	int motor_timer;
// timeout for loss of bluetooth
	int bt_timeout;
// timeout for loss of 433Mhz radio
	int radio_timeout;
// timeout for end of heading hold, to keep it on the same path after stopping
	int steering_timeout;
// maximum analog amount gyros can move while calculating center
	int gyro_center_max;
	float gyro_center;
	float prev_gyro_center;
	int need_gyro_center;
	int gyro_center_accum;
	int gyro_center_count;
	int have_gyro_center;
	int gyro_accum;
	int gyro_count;
	int gyro_min;
	int gyro_max;
	float max_gyro_drift;
	int ref_accum;
	int ref_count;


// RPM feedback mode only
	int target_rpm;
// reverse RPM
	int target_reverse_rpm;
// result of PID controller
	float throttle_feedback;
// the voltage reference
	float ref;
// the analog gyro reading
	float gyro;
// conversion factor for the gyro
	int angle_to_gyro;
// steering magnitude 0 - 100
	int max_steering100;
	int min_steering100;
// center position 0 - 100
	int mid_steering100;
// center position 0 - 100
	int mid_throttle100;



// PWM cycles between I steps
	int pid_downsample;
// purely gyro derived heading when not using mag, in radians
	float current_heading;
	float target_heading;
// steering with throttle
// number of PWM cycles for each steering step
	int steering_step_delay;
	int steering_step_counter;
// radians to increment heading after every step delay
	float steering_step;
// radians to rewind heading after manual steering
	float steering_overshoot;
// angle error before kicking the steering servo
//	float stiction_threshold;
// amount to kick the steering servo
//	float stiction_amount;
	
	
	
// enable heading hold
	int auto_steering;
// enable RPM feedback
	int auto_throttle;
	int headlights_on;
	int need_steering_feedback;

// PWM values recalculated in timer units
	int mid_steering_pwm;
	int mid_throttle_pwm;
	int min_throttle_reverse;
	int min_throttle_fwd;
	int max_steering_magnitude;
	int min_steering_magnitude;
	int throttle_ramp_step;





	int rpm_time;
	int rpm_status;
	int rpm_counter;
	int rpm;
	derivative_t rpm_dv;
	int rpm_dv_size;


    int spi_counter;
    int spi_state;
    int have_spi;
    uint32_t spi_buffer;
#define SPI_SYNC_CODE 0
#define SPI_PACKET 1

// enable the ak8975
	int enable_mag;
// enable path following
//	int enable_vision;
// where in the camera view the vanishing point should be 0 - 255
//    int vanish_center;
// where in the camera view the bottom of the center line should be 0 - 255
//	int bottom_center;
// raw data from vision program 0 - 255
//	int vanish_x;
//	int vanish_y;
//	int bottom_x;
//	float vanish_lowpass;
//	float bottom_lowpass;
//	float vision_bandwidth;
//	derivative_t path_dx;
//	int path_dx_size;
// delay between manual steering & path following
//	int manual_override_delay;
//	int manual_override_counter;

// wheel alignment statistics
    int throttle_accum;
    int throttle_count;
// values sent to phone
    int throttle_avg;
    int throttle_updated;

// radio statistics
    int bluetooth_packets;
    int stick_packets;
// values sent to phone
    int bluetooth_packets2;
    int stick_packets2;

// timer for updating statistics
	int stat_time;

// debugging
	int debug_counter;
	
	bluetooth_t bluetooth;
	nav_t nav;
} truck_t;

extern truck_t truck;










#endif





