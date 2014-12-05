/*
 * 1 handed truck
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
#include "arm_math.h"
#include "cc1101.h"
#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include <math.h>


// pass bluetooth to debug port
//#define BLUETOOTH_PASSTHROUGH
#define SYNC_CODE 0xe5


#define DEBUG_PIN GPIO_Pin_4
#define DEBUG_GPIO GPIOB

#define I_DOWNSAMPLE (NAV_HZ / 100)
// packets per second
#define PACKET_RATE 40

#define THROTTLE_MIN 0x100
#define THROTTLE_MAX 0xff00
// analog range
#define STEERING_MID 0x8000


#define BATTERY_OVERSAMPLE 1000
#define GYRO_OVERSAMPLE 20
#define GYRO_CENTER_TOTAL 4096

// TIM10 wraps at this frequency
#define TIMER_HZ 100
// gyro update rate
#define NAV_HZ 1024
// timer for LED flashing
#define LED_DELAY (NAV_HZ / 2)

// 50hz
#define PWM_PERIOD 1680000
// 2ms
#define MAX_PWM 168000
// 1ms
#define MIN_PWM 84000

truck_t truck;



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

void handle_radio()
{
	if(radio.packet[0] != SYNC_CODE) return;

	uint16_t chksum = get_chksum(radio.packet, PACKET_SIZE - 2);

	if((chksum & 0xff) == radio.packet[PACKET_SIZE - 2] &&
		((chksum >> 8) & 0xff) == radio.packet[PACKET_SIZE - 1])
	{
// packet good
		if(!truck.need_gyro_center)
		{
			truck.led_counter++;
			if(truck.led_counter >= LED_DELAY2)
			{
TRACE2
print_text("Begin gyro calibration\n");
				TOGGLE_PIN(LED_GPIO1, LED_PIN1);
				CLEAR_PIN(LED_GPIO2, LED_PIN2);
				truck.led_counter = 0;
			}
		}

		truck.throttle_reverse = radio.packet[1] & 0x1;
		truck.throttle = radio.packet[2] | (radio.packet[3] << 8);
		truck.steering = radio.packet[4] | (radio.packet[5] << 8);
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
			truck.need_gyro_center = 1;
			truck.gyro_center_count = 0;
			truck.gyro_accum = 0;
			truck.gyro_min = 65535;
			truck.gyro_max = -65535;
		}
		
		if(truck.have_gyro_center)
		{
			truck.throttle2 = truck.throttle;
			truck.throttle_reverse2 = truck.throttle_reverse;
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


#ifdef BLUETOOTH_PASSTHROUGH
static void bluetooth_passthrough()
{
	send_uart_binary(&truck.bluetooth.data, 1);
}
#endif // BLUETOOTH_PASSTHROUGH


void get_code1()
{
}

void handle_beacon()
{
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
		if(truck.shutdown_timeout > 0)
		{
			truck.shutdown_timeout--;
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
			truck.battery_accum = 0;
			truck.battery_count = 0;
			float voltage = truck.battery * 8.67f / 965.0f;

/*
 * TRACE2
 * print_number(truck.battery);
 * print_float(voltage);
 */


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
			int gyro = truck.gyro_accum / truck.gyro_count;
			truck.gyro_accum = 0;
			truck.gyro_count = 0;



//			TOGGLE_PIN(DEBUG_GPIO, DEBUG_PIN);


//			TRACE2
//			print_number(gyro);
			
			
			if(truck.need_gyro_center && !truck.have_gyro_center)
			{
				imu_led_flash();


				truck.gyro_center_accum += gyro;
				if(truck.gyro_center_count == 0)
				{
					truck.gyro_min = gyro;
					truck.gyro_max = gyro;
				}
				else
				{
					truck.gyro_min = MIN(gyro, truck.gyro_min);
					truck.gyro_max = MAX(gyro, truck.gyro_max);
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
					truck.gyro_min = 65535;
					truck.gyro_max = -65535;
				}
			}

			if(truck.need_gyro_center &&
				!truck.have_gyro_center && 
				truck.gyro_center_count >= GYRO_CENTER_TOTAL)
			{
				truck.gyro_center = (float)truck.gyro_center_accum / 
					truck.gyro_center_count;

				TRACE2
				print_number(truck.gyro_center_accum);
				print_number(truck.gyro_center_count);
				print_text("center=");
				print_float(truck.gyro_center);

				truck.have_gyro_center = 1;
				truck.need_gyro_center = 0;
			}
			
			if(truck.have_gyro_center)
			{
				truck.current_heading += (gyro - truck.gyro_center) / 
					truck.angle_to_gyro /
					NAV_HZ;
				truck.current_heading = fix_angle(truck.current_heading);
				truck.debug_counter++;
				if(truck.debug_counter >= 128)
				{
					truck.debug_counter = 0;
//					TRACE2
//					print_float(TO_DEG(truck.current_heading));
				}
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |
		GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
// must call this before ENABLE
	ADC_RegularChannelConfig(ADC1, 
		ADC_Channel_0, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC2, 
		ADC_Channel_1, 
		1, 
		ADC_SampleTime_480Cycles);
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
	ADC_SoftwareStartConv(ADC1);
	ADC_SoftwareStartConv(ADC2);

}

void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_FLAG_Update)
	{
		TIM2->SR = ~TIM_FLAG_Update;
		SET_PIN(GPIOA, GPIO_Pin_6);
		SET_PIN(GPIOA, GPIO_Pin_7);
	}

	if(TIM2->SR & TIM_FLAG_CC1)
	{
		TIM2->SR = ~TIM_FLAG_CC1;
		CLEAR_PIN(GPIOA, GPIO_Pin_6);
	}

	if(TIM2->SR & TIM_FLAG_CC2)
	{
		TIM2->SR = ~TIM_FLAG_CC2;
		CLEAR_PIN(GPIOA, GPIO_Pin_7);
	}
}

void init_pwm()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | 
		GPIO_Pin_7;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


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
	TIM_OCInitStructure.TIM_Pulse = MIN_PWM;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  	TIM_Cmd(TIM2, ENABLE);

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


int main(void)
{
	int i, j;


	init_linux();
	bzero(&truck, sizeof(truck_t));

	truck.steering = STEERING_MID;
	truck.gyro_center_max = 100;
	truck.angle_to_gyro = 450;

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

	print_text("Welcome to the truck\n");
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


	init_bluetooth();
	init_analog();
	init_pwm();


	init_cc1101();
	cc1101_receiver();

// DEBUG
// bypass calibration for testing
//	imu.have_gyro_center = 1;

// heading error -> PWM
	init_pid(&truck.heading_pid, 
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
		handle_bluetooth();
		handle_analog();
//		handle_timer1();
		if(radio.got_packet)
		{
			radio.got_packet = 0;
			handle_radio();
		}





	}
	
		
}





