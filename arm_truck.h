#ifndef ARM_TRUCK_H
#define ARM_TRUCK_H

#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"


// timer for packet flashing
#define LED_DELAY2 2
// green pin when high
#define LED_PIN1 GPIO_Pin_4
#define LED_GPIO1 GPIOC
// red pin when high
#define LED_PIN2 GPIO_Pin_5
#define LED_GPIO2 GPIOC

typedef struct
{
	float p_gain;
	float i_gain;
	float d_gain;
	float i_limit;
	float o_limit;
	int counter;
	float error_accum;
	float accum;
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



typedef struct
{
	int timer_high;
// data from radio	
	int throttle_reverse;
	int throttle_reverse2;
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
// throttle starts at this level when using feedback
	int throttle_base;
	int throttle_state;
#define THROTTLE_OFF 0 
#define THROTTLE_RAMP 1
#define THROTTLE_AUTO 2

	pid_t heading_pid;
	pid_t throttle_pid;
	pid_t rpm_pid;

	int battery;
	int battery_accum;
	int battery_count;
// voltage in V
	float battery_voltage;
// ADC value of battery voltage
	int battery_analog;
	float battery_v0;

// steering was pressed before throttle
	int steering_first;
// Make the interrupt handler not copy the radio to the steering PWM
	int writing_settings;
	int motor_timer;
	int bt_timeout;
	int radio_timeout;
	int need_gyro_center;
	int gyro_center_accum;
	int gyro_center_count;
	int have_gyro_center;
	int gyro_accum;
	int gyro_count;
	int gyro_min;
	int gyro_max;
	int ref_accum;
	int ref_count;
	int current_accum;
	int current_count;
// ADC value of current based on voltage drop
	float raw_current;
// measured current in A
	float current;
// power in W based on battery voltage & current
	float power;
// power feedback mode only
	float target_power;
// wattage of no rpm_slope
	float power_base;
// power above & below power_base makes the RPM fall & rise by this amount per watt
	float rpm_slope;
// RPM feedback mode only
	int target_rpm;
// RPM feedback after rolling back for power
	int target_rpm2;
// result of PID controller
	float throttle_feedback;
// currently sampling the ref pin
	int sample_ref;
// the voltage reference
	float ref;
// the analog gyro reading
	float gyro;
// maximum analog amount gyros can move while calculating center
	int gyro_center_max;
	float gyro_center;
	int angle_to_gyro;
// throttle magnitude 0 - 100
	int max_throttle_fwd;
	int max_throttle_rev;
// steering magnitude 0 - 100
	int max_steering;
	int min_steering;
// center position 0 - 100
	int mid_steering;
// center position 0 - 100
	int mid_throttle;
// number of PWM cycles for each throttle step
	int throttle_ramp_delay;
	int throttle_ramp_counter;
	int throttle_ramp_step;
// PWM cycles between I steps
	int pid_downsample;
// in radians
	float current_heading;
// steering with throttle
// number of PWM cycles for each steering step
	int steering_step_delay;
	int steering_step_counter;
// radians to increment heading after every step delay
	float steering_step;
// radians to rewind heading after manual steering
	float steering_overshoot;
// enable heading hold
	int auto_steering;
	int headlights_on;

	int rpm_time;
	int rpm_status;
	int rpm_counter;
	int rpm;
	int throttle_time;

	
	int debug_counter;
	
	bluetooth_t bluetooth;
} truck_t;

extern truck_t truck;










#endif





