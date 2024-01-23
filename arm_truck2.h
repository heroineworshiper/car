/*
 * STM32 Controller for direct drive truck
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


#ifndef ARM_TRUCK2_H
#define ARM_TRUCK2_H

#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "arm_math.h"
//#include "arm_hardi2c.h"
#include "arm_softi2c.h"





// reverse steering servo
//#define REVERSE_STEERING
// reverse analog values
#define REVERSE_STEERING_ADC
#define REVERSE_THROTTLE_ADC
// pass bluetooth to debug port
//#define BLUETOOTH_PASSTHROUGH
// before a device is configured, its UART will be 9600
//#define BLUETOOTH9600 


// smart leash
#define USE_LEASH
// XY feedback.  Change Truck.java if you change this.
#define LEASH_XY
#define LEASH_PROTOCOL2

#define HALLS 4
#define MOTORS 2

#define LEFT_MOTOR 0
#define RIGHT_MOTOR 1
#define LEFT_HALL 0
#define RIGHT_HALL 2
 

// gyro update rate
#define NAV_HZ 430

// tick/TIM10 wraps at this frequency
// how fast the feedback loop fires
#define TIMER_HZ 100

// how fast the PWM fires
#define PWM_HZ 100
// what the feedback is calibrated for
#define PWM_BASE 50

// how fast the RPM is updated
#define RPM_HZ 10

// motors
#define MOTOR_PWM_PERIOD 409
#define HALL_OVERSAMPLE 2
#define ANGLE_STEP 40

// memory resident table creation
#define LINES (360 * 7 / ANGLE_STEP)
extern uint16_t motor_lines[LINES][4];


#define PACKET_SIZE 8

// timer for packet flashing
#define LED_DELAY2 2
// red LED when high
#define RED_LED GPIO_Pin_14
// green LED when high
#define GREEN_LED GPIO_Pin_13
#define LED_GPIO GPIOA


// motor enable pins
#define LEFT_EN1_PIN GPIO_Pin_1
#define LEFT_EN1_GPIO GPIOC
#define LEFT_EN2_PIN GPIO_Pin_2
#define LEFT_EN2_GPIO GPIOC
#define LEFT_EN3_PIN GPIO_Pin_3
#define LEFT_EN3_GPIO GPIOA

#define RIGHT_EN1_PIN GPIO_Pin_2
#define RIGHT_EN1_GPIO GPIOB
#define RIGHT_EN2_PIN GPIO_Pin_10
#define RIGHT_EN2_GPIO GPIOB
#define RIGHT_EN3_PIN GPIO_Pin_11
#define RIGHT_EN3_GPIO GPIOB





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

	int ignore_i; // don't wind up if over the output limit

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

// filter order
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
	void (*current_function)();
	unsigned char data;
	unsigned char packet[PACKET_SIZE];
	unsigned char next_packet[PACKET_SIZE];
	int got_packet;
	int counter;

} radio_t;

// hall effect sensor
typedef struct
{
	void (*current_function)(void *);
// the readings
    int accum;
    int readings;
// the averaged reading
    int value;
} hall_t;


typedef struct
{
	void (*current_function)(void *ptr);
// uncentered, lowpassed values
	float gyro_z;
	float gyro_z_centered;
	float gyro_bandwidth;
	float gyro_z_center;
	float prev_gyro_z_center;
	float gyro_z_accum;
	int gyro_center_count;
	int gyro_z_min;
	int gyro_z_max;
	float abs_heading;
	float current_heading;
	int total_gyro;
	i2c_t i2c;
} imu_t;


// motor state
typedef struct
{
// last sensed position in absolute degrees
    int angle;
// last angle for computing direction in absolute degrees
    int ref_angle;
// last direction detected
    int reverse;
// driven position in degrees in the current pole
    int phase;
    int prev_phase;
// angular velocity in degrees/second
    int v;
    int v_temp;
} motor_t;

#ifdef USE_LEASH
typedef struct
{
    uint8_t buffer[8];
// offset in the packet
    int offset;
    int got_esc;
    int got_start;
// raw data
    int angle_adc;
    int encoder0_adc;
    int encoder1_adc;
// parameters for the magnetic angle sensor
    float min_angle;
    float max_angle;
//    int min_angle_adc;
//    int max_angle_adc;
//    int center_angle_adc;
// leash angle in rads
    float angle;
// length read from serial port
    int length;
// latest ground distance in encoder counts
    float distance;
// previous ground distance
    int distance2;
    int timeout;
    int active;

#ifdef LEASH_XY
// X Y in encoder counts
    float x;
    float y;
#endif // LEASH_XY


// encoder count to start moving at
    int distance0;
// starting speed in RPM
    float rpm0;
// minutes per mile per encoder count
    float speed_to_distance;
// minutes per mile maximum speed
    float max_speed;
// center leash angle in rads
    float center;
// X offset user can manually add or subtract from center
// in encoder counts if LEASH_XY or rads
    float x_offset;
// number of the current offset: -1, 0, 1
    int current_offset;
// last binary steering value for adjusting offset
    int stick_state;
// the PID controller
	pid_t steering_pid;
	derivative_t steering_d;
	int steering_d_size;
    float steering_i_limit;
    float steering_d_limit;

	filter_t error_highpass;
// 0 - 100
//	float highpass_bandwidth;
    filter_t error_lowpass;
    float lowpass_bandwidth;
    float highpass_bandwidth;
} leash_t;
#define LEASH_TIMEOUT TIMER_HZ
#endif // USE_LEASH



typedef struct
{
// the mane timer which increments at TIMER_HZ
	int tick;
    int feedback_tick;
// next time to update the tachometer
    int rpm_tick;
    int debug_tick;
// a time used by the throttle
	int start_time;
// current remote value
	int throttle;
	int steering;
    int binary_steering;
    int speed_offset;
// controller is in calibration mode
    int calibration_mode;
// last throttle value when the motor moved
	int throttle2;
	int steering2;
    int speed_offset2;

	int bt_throttle;
	int bt_steering;
	int have_bt_controls;

// PWM value
	int steering_pwm;
	int led_counter;






	pid_t heading_pid;
	filter_t error_highpass;
// 0 - 100
	float highpass_bandwidth;

	pid_t rpm_pid;
	imu_t imu;

	int battery;
	int battery_accum;
	int battery_count;
// voltage in V
	float battery_voltage;
// ADC value of battery voltage
	int battery_analog;
	float battery_v0;

// Make the interrupt handler not copy the radio to the steering PWM
	int writing_settings;
	int motor_timer;
// timeout for loss of bluetooth
	int bt_timeout;
// timeout for loss of proprietary radio
	int radio_timeout;

// timeout for end of heading hold, to keep it on the same path after stopping
	int steering_timeout;
// maximum analog amount gyros can move while calculating center
	int gyro_center_max;
	float gyro_center;
	float prev_gyro_center;
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


// the target forward speed from config file
    int target_rpm;
// maximum reverse RPM from config file
	int target_reverse_rpm;
// wheel diameter in mm
    int diameter;
// result of PID controller
	float throttle_feedback;
// the voltage reference
	float ref;
// the analog gyro reading
	float gyro;
// conversion factor for the gyro.  Reverse it to reverse the gyro sign.
	int angle_to_gyro;

// PWM range
// steering magnitude 0 - 100
	int max_steering100;
// center position 0 - 100
	int mid_steering100;
// minimum stick induced steering 0 - 100
    int min_steering100;
// maximum stick induced throttle 0 - 100
    int throttle_base100;
    int throttle_reverse_base100;
// minimum stick induced throttle 0 - 100
	int min_throttle_fwd100; // NEW
	int min_throttle_reverse100; // NEW

// remote control ranges
    int remote_steering_mid;
    int remote_steering_deadband;
    int remote_steering_max;
    int remote_steering_min;
    
    int remote_throttle_mid;
    int remote_throttle_deadband;
    int remote_throttle_max;
    int remote_throttle_min;

// PWM cycles between I steps
	int pid_downsample;
// gyro derived heading, in radians
	float current_heading;
	float target_heading;
// automated steering
// slowest radians per second
	float min_steering_step;
// fastest radians per second // NEW
	float max_steering_step;
// radians to rewind heading after manual steering
	float steering_overshoot;
	
	
	
// enable heading hold
	int auto_steering;
	int need_steering_feedback;

// percent values converted to PWM timer values
	int mid_steering_pwm;
	int max_steering_magnitude;
	int min_steering_magnitude;
	int throttle_ramp_step;

// common motor power 0 - MOTOR_PWM_PERIOD
    int power;
// motor direction
    int reverse;
// enable RPM feedback
    int auto_throttle;



	int rpm;
	derivative_t rpm_dv;
	int rpm_dv_size;


    int spi_counter;
    int spi_state;
    int have_spi;
    uint32_t spi_buffer;
#define SPI_SYNC_CODE 0
#define SPI_PACKET 1


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

#define START_TEST 0
#define TEST_PASS1 1
#define TEST_PASS2 2
#define TEST_DONE 3
    int test_state;
    int test_read;
    int test_phase;
    int test_tick;
    int testing_motors;

	bluetooth_t bluetooth;
    hall_t halls[HALLS];
// current ADC channel for the halls
    int current_hall;
    motor_t motors[MOTORS];
    radio_t radio;
} truck_t;

extern truck_t truck;
#ifdef USE_LEASH
extern leash_t leash;
#endif










#endif





