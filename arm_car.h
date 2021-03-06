#ifndef ARM_CAR_H
#define ARM_CAR_H

#include "stm32f4xx_adc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

#define MOTORS 2
#define PHASES 3
#define SENSORS 3

// timer for packet flashing
#define LED_DELAY2 2
#define LED_PIN GPIO_Pin_10
#define LED_GPIO GPIOB

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

typedef struct
{
	TIM_TypeDef *pwm_timer;
	ADC_TypeDef *adc;
	uint16_t in_pin[PHASES];
	GPIO_TypeDef *in_gpio[PHASES];
	uint16_t en_pin[PHASES];
	GPIO_TypeDef *en_gpio[PHASES];

// 0 - 2
	int adc_channel;
	int adc_channels[SENSORS];
	int adc_results[SENSORS];
	int adc_min[SENSORS];
	int adc_max[SENSORS];
	int adc_throwaway;
// filtered sensor values
	int sensors[SENSORS];
// index of table
	int sensor_order;
// hall effect sensor threshold
	int threshold;
	
// 0 - MAX_PWM
	int power;
// direction to move phase for a right turn
	int turn_sign;
// phase of each motor (0 - 360)
	float phase;
// raw PWM for each phase
	int pwm[PHASES];
// directions of motors
	int commutation_direction;
	int stepper_direction;
// ramp for commutation
	int do_ramp;

// what phases are floating in freewheel mode
	int disabled[PHASES];
// phase voltage in freewheel mode
	int voltage[PHASES];
// last slot in table used
	int commutation_slot;
// debugging
	int commutations;
} motor_t;

typedef struct
{
	int timer_high;
	
	int throttle_reverse;
	int throttle;
	int throttle_reverse2;
	int throttle2;
	int steering;
	int led_counter;
	
	pid_t heading_pid;
// radians
	float target_pitch;
	float total_step;
	
	int battery;
	int battery_accum;
	int battery_count;
	int got_battery;
	
	int motor_timer;
	int shutdown_timeout;
	int stepper_power;
//	int operating_power0;
	int operating_power;


	motor_t motor[MOTORS];
// motor mode
	int mode;
// time between steps
	int period;

	int test_motors;
// debugging value
	float period_lowpass;
	int debug_counter;
} car_t;

extern car_t car;










#endif





