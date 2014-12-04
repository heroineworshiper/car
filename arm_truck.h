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

// PWM values
	int throttle_pwm;
	int steering_pwm;
	int led_counter;
	
	pid_t heading_pid;
	
	int battery;
	int battery_accum;
	int battery_count;
	
	int motor_timer;
	int shutdown_timeout;
	int need_gyro_center;
	int have_gyro_center;
	int gyro_accum;
	int gyro_count;
	
	bluetooth_t bluetooth;
} truck_t;

extern truck_t truck;










#endif





