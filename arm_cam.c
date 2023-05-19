/*
 * STM32 CONTROLLER FOR CAMERA PANNER
 * Copyright (C) 2020-2023 Adam Williams <broadcast at earthling dot net>
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

// make cam.bin;./uart_programmer cam.bin

// The camera firmware takes commands over USB from a face tracker or 
// an RF remote.
// If the remote is on, the face tracker is ignored.

// This version sends UART codes to a brushless servo programmed to be 
// a stepper motor.


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
#include "arm_usb.h"


// settings
// time
#define HZ 1000

// use A14 CLK & A13 DATA for external SPI commands
//#define USE_SPI
#define USE_USB

// time before shutting motor down
#define MOTOR_TIMEOUT (HZ / 4)



// analog ranges
#define ADC_CENTER 114
#define ADC_DEADBAND 5
// minimums
#define MIN_LEFT (ADC_CENTER + ADC_DEADBAND)
#define MIN_RIGHT (ADC_CENTER - ADC_DEADBAND)
// maximums
#define MAX_LEFT 210
#define MAX_RIGHT 40

#define FRACTION 256

#define MAX_STEP (24 * FRACTION)
#define MIN_STEP (FRACTION)
#define ACCELERATION (8 * FRACTION / 256)

// 20x timelapse
#define TIMELAPSE_STEP (MAX_STEP / 80)

//#define DEBUG_PIN GPIO_Pin_4
//#define DEBUG_GPIO GPIOB

#define LED_PIN GPIO_Pin_0
#define LED_GPIO GPIOB



// SPI
#define CLK_PIN GPIO_Pin_14
#define DAT_PIN GPIO_Pin_13
#define SPI_GPIO GPIOA

// idle ticks before resetting SPI
#define SPI_TICKS 10
// ticks before timing out adc_spi
#define SPI_TIMEOUT (HZ / 2)

#ifdef USE_SPI
volatile int last_spi_tick = 0;
volatile uint8_t spi_byte = 0;
volatile uint8_t spi_counter = 0;
volatile uint8_t spi_len = 0;
volatile uint8_t spi_data[8];
volatile uint8_t adc_spi = 0xff;
volatile int spi_valid = 0;
#endif

#ifdef USE_USB
volatile int last_usb_tick = 0;
volatile uint8_t adc_usb = 0xff;
volatile int usb_valid = 0;
#endif



// frequency hopping rate
#define HOP_HZ 25
// rate when scanning
#define SCAN_HZ 5
// time before next packet we should hop (10ms)
#define HOP_LAG (HZ / 100)

const uint8_t PACKET_KEY[] = 
{
    0x5e, 0x1b, 0xdb, 0xc8, 0x98, 0xa1, 0x5e, 0x90
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0x00, 0x00, 0xaa, 0xaa, 0x55, 0x55
};

#define PACKET_DATA 8

// frequency hopping table.  64 frequency steps = 480khz 
// Check temp_sensor.X & cam_remote.X for taken frequencies
// 901-928Mhz
#define MAX_FREQ 3839
#define MIN_FREQ 160
#define FREQ_RANGE (MAX_FREQ - MIN_FREQ)
// freq hopping.  Offset the traction controller frequencies.
const uint16_t channels[] = 
{
    MIN_FREQ + FREQ_RANGE / 16, 
    MAX_FREQ - FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 4 / 8 + FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 2 / 8 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 6 / 8 - FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 1 / 8 + FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 7 / 8 - FREQ_RANGE / 16, 
    MIN_FREQ + FREQ_RANGE * 3 / 8 + FREQ_RANGE / 16,
    MIN_FREQ + FREQ_RANGE * 5 / 8 - FREQ_RANGE / 16, 
};

#define TOTAL_CHANNELS (sizeof(channels) / sizeof(uint16_t))



// RADIO_CHANNEL is from 96-3903 & set by the user
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) / (1 + DRPE * 7) - 1
// RADIO_DATA_SIZE is the amount of data to read before resetting the sync code

//#define RADIO_CHANNEL 96

// scan for synchronous code
#define FIFORSTREG 0xCA81
// read continuously
//#define FIFORSTREG              (0xCA81 | 0x0004)
// 915MHz
#define FREQ_BAND 0x0030
// Center Frequency: 915.000MHz
//#define CFSREG (0xA000 | RADIO_CHANNEL)
#define CFSREG(chan) (0xA000 | (chan))
// crystal load 10pF
#define XTAL_LD_CAP 0x0003
// power management page 16
#define PMCREG 0x8201
#define GENCREG (0x8000 | XTAL_LD_CAP | FREQ_BAND)


// +3/-4 Fres
//#define AFCCREG 0xc4f7
// +15/-16 Fres
#define AFCCREG 0xc4d7

// Data Rate
// data rate must be slow enough to service FIFOs
// kbps = 10000 / (29 * (DRVSREG<6:0> + 1) * (1 + DRPE * 7))
// RADIO_BAUD_CODE = 10000 / (29 * kbps) - 1
#define RADIO_BAUD_CODE 3

// data rate prescaler.  Divides data rate by 8 if 1
//#define DRPE (1 << 7)
#define DRPE 0
#define DRVSREG (0xC600 | DRPE | RADIO_BAUD_CODE)


// Page 37 of the SI4421 datasheet gives optimum bandwidth values
// but the lowest that works is 200khz
//#define RXCREG 0x9481     // BW 200KHz, LNA gain 0dB, RSSI -97dBm
//#define RXCREG 0x9440     // BW 340KHz, LNA gain 0dB, RSSI -103dBm
#define RXCREG 0x9420       // BW 400KHz, LNA gain 0dB, RSSI -103dBm

//#define TXCREG 0x9850     // FSK shift: 90kHz
#define TXCREG 0x98f0       // FSK shift: 165kHz
#define STSREG 0x0000
#define RXFIFOREG 0xb000

// analog filter for raw mode
#define BBFCREG                 0xc23c

volatile int current_channel = 0;
volatile int missed_packets = 0;
// start time of last received packet
volatile int packet_tick = 0;
// time of next hop
volatile int next_hop = 0;
volatile int need_hop = 0;
volatile int scanning = 0;
#define MAX_MISSED_PACKETS HOP_HZ





#define ABS(x) ((x) > 0 ? (x) : (-(x)))

// 0 - SIN_TOTAL * FRACTION
int phase = 0;
// phase change per frame * FRACTION
int step = 0;
int goal_step = 0;

// motor codes
#define GROUND_MOTOR 0xfe
#define COAST_MOTOR 0xff
#define SIN_TOTAL 252
int motor_enabled = 0;

#define MAX_POWER 0xc0
#define MIN_POWER 0x20
int motor_power = 0xc0;

// motor UART
#define MOTOR_PACKET 4
uint8_t motor_packet[MOTOR_PACKET];
int motor_offset = MOTOR_PACKET;


// last wanted change per frame
int target_phase_speed = 0;
int motor_timeout = 0;

volatile int tick = 0;


// radio parsing
void (*radio_function)();
#define RADIO_BUFSIZE (sizeof(PACKET_KEY) + PACKET_DATA)
volatile unsigned char receive_buf[RADIO_BUFSIZE];
volatile int radio_counter = 0;
volatile unsigned char radio_data = 0;
#define RADIO_TIMEOUT HZ
volatile int timeout_counter = 0;
// values from the radio
volatile uint8_t adc_raw = 0xff;
volatile uint8_t timelapse_code = 0xff;
volatile uint8_t control_valid = 0;
volatile uint8_t spi_valid = 0;
RCC_ClocksTypeDef RCC_ClocksStatus;

#define RADIO_CS_GPIO GPIOB
#define RADIO_CS_PIN GPIO_Pin_12
#define RADIO_SDO_GPIO GPIOB
#define RADIO_SDO_PIN GPIO_Pin_15
#define RADIO_CLK_GPIO GPIOB
#define RADIO_CLK_PIN GPIO_Pin_13

// write motor phase to hardware
void write_motor()
{
    if(motor_offset >= MOTOR_PACKET)
    {
        motor_offset = 0;
        motor_packet[0] = 0xff;
        motor_packet[1] = 0x88;
        motor_packet[2] = motor_power;
        if(motor_enabled)
            motor_packet[3] = phase / FRACTION;
        else
            motor_packet[3] = COAST_MOTOR;
    }
}



void get_key();
void get_packet()
{
    receive_buf[radio_counter++] = radio_data;
    if(radio_counter >= PACKET_DATA)
    {
        radio_counter = 0;
        radio_function = get_key;
        
        int i;
        int failed = 0;

// XOR the data key
        for(i = 0; i < PACKET_DATA; i++)
        {
            receive_buf[i] ^= DATA_KEY[i];
        }

        for(i = 2; i < PACKET_DATA; i += 2)
        {
// reject an invalid packet
            if(receive_buf[i] != receive_buf[0] ||
                receive_buf[i + 1] != receive_buf[1])
            {
                failed = 1;
                break;
            }
        }

        if(!failed)
        {
//TRACE2
//print_text("tick=");
//print_number(tick);
// print_number(receive_buf[0]);
// print_text("blink_counter=");
// print_number(receive_buf[1]);
// print_lf();

            TOGGLE_PIN(LED_GPIO, LED_PIN);
            timeout_counter = 0;
//print_text("got chan=");
//print_number(current_channel);
//print_lf();
            timelapse_code = receive_buf[0];
            adc_raw = receive_buf[1];
            control_valid = 1;
TRACE2
print_text("timelapse_code=");
print_number(timelapse_code);
print_text("adc_raw=");
print_number(adc_raw);

// schedule the next hop to the start time of this packet + HOP duration + HOP_LAG
            scanning = 0;
            missed_packets = 0;
            next_hop = packet_tick + HZ / HOP_HZ - HOP_LAG;
        }
    }
}

void get_key()
{
    if(radio_data == PACKET_KEY[radio_counter])
    {
        if(radio_counter == 0)
        {
            packet_tick = tick;
        }
    
        radio_counter++;
        if(radio_counter >= sizeof(PACKET_KEY))
        {
            radio_function = get_packet;
            radio_counter = 0;
        }
    }
    else
    if(radio_data == PACKET_KEY[0])
    {
        packet_tick = tick;
        radio_counter = 1;
    }
    else
    {
        radio_counter = 0;
    }
}

// write radio SPI.  TODO: use hardware
void write_radio(uint16_t data)
{
// print_text("write_radio ");
// print_hex(data);
// print_lf();
    CLEAR_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);

    int i;
    for(i = 0; i < 16; i++)
    {
        if(data & 0x8000)
        {
            SET_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        else
        {
            CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
        }
        data <<= 1;
// need the delay if the system clock is over 16Mhz
//        udelay(1);
        SET_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
//        udelay(1);
        CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    }
    
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
}

void init_radio()
{
    TRACE
    GPIO_InitTypeDef GPIO_InitStructure;
	radio_function = get_key;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#define RADIO_RX_PIN 11
#define RADIO_TX_PIN 10
	GPIO_PinAFConfig(GPIOB, RADIO_RX_PIN, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, RADIO_TX_PIN, GPIO_AF_USART3);

// RX enabled
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = 1 << RADIO_RX_PIN;
  	GPIO_Init(GPIOB, &GPIO_InitStructure);

// TX enabled
//  GPIO_InitStructure.GPIO_Pin = 1 << RADIO_TX_PIN;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 100000;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx /* | USART_Mode_Tx */;
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

// initialize radio SPI
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RADIO_CS_PIN |
        RADIO_SDO_PIN |
        RADIO_CLK_PIN;
    GPIO_Init(RADIO_CS_GPIO, &GPIO_InitStructure);
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
    CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
    CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    
    



// scan for synchronous code
    write_radio(FIFORSTREG);
// enable synchron latch
    write_radio(FIFORSTREG | 0x0002);
    write_radio(GENCREG);
    write_radio(AFCCREG);
    write_radio(CFSREG(channels[current_channel]));
    write_radio(DRVSREG);
    write_radio(PMCREG);
    write_radio(RXCREG);
    write_radio(TXCREG);
    write_radio(BBFCREG);
// turn on the transmitter to tune.  Not necessary to receive.
//    write_radio(PMCREG | 0x0020);

// warm up
//     int tick1 = tick;
//     while(tick - tick1 < HZ / 100)
//     {
//         ;
//     }

// receive mode
    write_radio(PMCREG | 0x0080);

}



void next_channel()
{
    current_channel++;
    if(current_channel >= TOTAL_CHANNELS)
    {
        current_channel = 0;
    }
//print_text("tune chan=");
//print_number(current_channel);
//print_lf();
    write_radio(CFSREG(channels[current_channel]));
}







void init_motor()
{
// transmit pin for motor UART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_ResetBits(GPIOA, GPIO_InitStructure.GPIO_Pin);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;

	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
/* USART configuration */
  	USART_Init(USART2, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART2, ENABLE);


    write_motor();

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



// DEBUG uart
void USART6_IRQHandler(void)
{
	unsigned char c = USART6->DR;
	uart.input = c;
	uart.got_input = 1;
}

// radio uart
void USART3_IRQHandler(void)
{
	radio_data = USART3->DR;
	radio_function();
}



// TIM10 wraps at HZ
void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
		
//        TOGGLE_PIN(LED_GPIO, LED_PIN);

		tick++;


// reset the control code
        if(timeout_counter >= RADIO_TIMEOUT)
        {
            control_valid = 0;
        }
        else
        {
            timeout_counter++;
        }


        if(tick >= next_hop)
        {
            need_hop = 1;
        }

	}
}

#ifdef USE_SPI
void EXTI15_10_IRQHandler()
{
    if((EXTI->PR & EXTI_Line14))
    {
// reset it
        EXTI->PR = EXTI_Line14;
        if(PIN_IS_SET(SPI_GPIO, CLK_PIN))
        {
// reset after inactivity
            if(tick - last_spi_tick > SPI_TICKS)
            {
                spi_counter = 0;
                spi_byte = 0;
                spi_len = 0;
            }

            last_spi_tick = tick;
            spi_byte <<= 1;
            if(PIN_IS_SET(SPI_GPIO, DAT_PIN))
            {
                spi_byte |= 0x1;
            }

            spi_counter++;
            if(spi_counter >= 8)
            {
                if(spi_len < sizeof(spi_data))
                {
                    spi_data[spi_len++] = spi_byte;
                }

                spi_counter = 0;
                spi_byte = 0;
            }
        }
    }
}

#endif




#ifdef USE_USB
extern unsigned char *in_buf;
uint8_t class_cb_DataOut (void  *pdev, uint8_t epnum)
{
print_buffer(in_buf, 64);
    if(!control_valid)
    {
        TOGGLE_PIN(LED_GPIO, LED_PIN);
    }
    last_usb_tick = tick;
    adc_usb = in_buf[0];
    usb_valid = 1;
//            TRACE
//            print_hex(adc_spi);
  return 0;
}
#endif // USE_USB



int main(void)
{
	int i, j;
 
 
 
// switch system clock to non PLL
    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
// disable mane PLL
//    RCC_PLLCmd(DISABLE);

// clockspeed = 8Mhz / PLLM * PLLN / PLL_P
// 168Mhz = 63mA
// 32Mhz = 45mA
// reduce clockspeed to reduce power consumption
// mane PLL
#define PLL_M 8
#define PLL_N 32
#define PLL_P 2
#define PLL_Q 7
//     RCC->PLLCFGR = PLL_M | 
// 		(PLL_N << 6) | 
// 		(((PLL_P >> 1) -1) << 16) |
//         (RCC_PLLCFGR_PLLSRC_HSI) | 
// 		(PLL_Q << 24);
//     RCC_PLLCmd(ENABLE);



    SystemCoreClockUpdate();

	init_linux();

/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB, 
		ENABLE);

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();


	RCC_GetClocksFreq(&RCC_ClocksStatus);
	print_text("Welcome to cam panner\n");
	flush_uart();
	print_text("SYSCLK_Frequency=");
	print_number(RCC_ClocksStatus.SYSCLK_Frequency);
	print_lf();
	print_text("SystemCoreClock=");
	print_number(SystemCoreClock);
	print_lf();
	print_text("HCLK_Frequency=");
	print_number(RCC_ClocksStatus.HCLK_Frequency);
	print_lf();
	print_text("PCLK1_Frequency=");
	print_number(RCC_ClocksStatus.PCLK1_Frequency);
	print_lf();
	print_text("PCLK2_Frequency=");
	print_number(RCC_ClocksStatus.PCLK2_Frequency);
	print_lf();

	flush_uart();
    init_watchdog();
	flush_uart();

// general purpose timer
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	TIM_DeInit(TIM10);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = RCC_ClocksStatus.SYSCLK_Frequency / 100 / HZ - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 100;
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


	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;

// debug pin
// 	GPIO_InitStructure.GPIO_Pin = DEBUG_PIN;
// 	GPIO_Init(DEBUG_GPIO, &GPIO_InitStructure);
// 	CLEAR_PIN(DEBUG_GPIO, DEBUG_PIN);

// LED pin
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	SET_PIN(LED_GPIO, LED_PIN);


#ifdef USE_SPI
// SPI from raspberry pi
    GPIO_InitStructure.GPIO_Pin = CLK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SPI_GPIO, &GPIO_InitStructure);
    
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line14;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

 	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
 	NVIC_Init(&NVIC_InitStructure);



#endif

	init_motor();
    init_radio();

// wait a while
    while(tick < HZ)
    {
        ;
    }
	init_usb();

    int seconds = 0;
    int prev_frame = 0;
    int debug_count = 0;
	while(1)
	{
		handle_uart();
        handle_usb();

// transmit motor code
	    if((USART2->SR & USART_FLAG_TC) != 0)
        {
            if(motor_offset < MOTOR_PACKET)
            {
                USART2->DR = motor_packet[motor_offset++];
            }
        }

        if(tick != prev_frame)
        {
            prev_frame = tick;
            debug_count++;
            debug_count = debug_count % HZ;

// update speed based on remote value
            int adc = adc_raw;


// use face tracker
#ifdef USE_SPI
            if(!control_valid && 
                tick - last_spi_tick < SPI_TIMEOUT)
            {
                adc = adc_spi;
            }
            else
            {
                spi_valid  = 0;
            }
#endif

#ifdef USE_USB
            if(!control_valid &&
                tick - last_usb_tick < SPI_TIMEOUT)
            {
                adc = adc_usb;
            }
            else
            {
                usb_valid = 0;
            }
#endif


            goal_step = 0;
            if(control_valid || spi_valid || usb_valid)
            {
                if(adc < MIN_RIGHT)
                {
                    goal_step = -MIN_STEP - (MAX_STEP - MIN_STEP) * 
                        (MIN_RIGHT - adc) /
                        (MIN_RIGHT - MAX_RIGHT);
                    CLAMP(goal_step, -MAX_STEP, 0);
                }
                else
                if(adc > MIN_LEFT)
                {
                    goal_step = MIN_STEP + (MAX_STEP - MIN_STEP) * 
                        (adc - MIN_LEFT) /
                        (MAX_LEFT - MIN_LEFT);
                    CLAMP(goal_step, 0, MAX_STEP);
                }
                else
                if(control_valid && timelapse_code < 2)
                {
// handle the timelapse code
                    if(timelapse_code == 0)
                        goal_step = TIMELAPSE_STEP;
                    else
                    if(timelapse_code == 1)
                        goal_step = -TIMELAPSE_STEP;
                }
            }




// apply acceleration
            if(goal_step > step) 
            {
                step += ACCELERATION;
                CLAMP(step, -MAX_STEP, MAX_STEP);
            }
            else
            if(goal_step < step)
            {
                step -= ACCELERATION;
                CLAMP(step, -MAX_STEP, MAX_STEP);
            }

// handle timeout
            if(step != 0)
            {
                motor_timeout = 0;
                motor_enabled = 1;
            }
            else
            {
// disable motor
                if(motor_timeout >= MOTOR_TIMEOUT)
                    motor_enabled = 0;
                else
                    motor_timeout += 1;
            }


            if(motor_enabled)
            {
                motor_power = MIN_POWER + 
                    ABS(step) * (MAX_POWER - MIN_POWER) / MAX_STEP;
                phase += step;
                while(phase < 0)
                    phase += SIN_TOTAL * FRACTION;
                while(phase >= SIN_TOTAL * FRACTION)
                    phase -= SIN_TOTAL * FRACTION;
            }

            write_motor();



            if(debug_count == 0)
            {
                 print_text("adc=");
                 print_number(adc);
                 print_text("timelapse=");
                 print_number(timelapse_code);
                 print_text("step=");
                 print_fixed(step);
                 print_text("power=");
                 print_number(motor_power);
                 print_text("enabled=");
                 print_number(motor_enabled);
                 print_lf();
            }

        }


// check clock
        if(tick / HZ != seconds)
        {
            seconds = tick / HZ;
//             print_text("seconds=");
//             print_number(seconds);
//             print_lf();
        }



// frequency hopping
        if(need_hop)
        {
            need_hop = 0;
            if(scanning)
            {
                next_hop = tick + HZ / SCAN_HZ;
            }
            else
            {
// always incremented, whether we got a packet or not
                missed_packets++;
// too many hops without a packet.  Go to scanning mode
                if(missed_packets > MAX_MISSED_PACKETS)
                {
                    scanning = 1;
                    next_hop = tick + HZ / SCAN_HZ;
                }
                else
                {
                    next_hop = tick + HZ / HOP_HZ;
                }
            }

//             if(missed_packets > 1)
//             {
//                 print_text("missed chan=");
//                 print_number(current_channel);
//                 print_lf();
//             }
//TRACE2
//print_text("scanning=");
//print_number(scanning);
//print_text("current_channel=");
//print_number(current_channel);
            next_channel();
        }

#ifdef USE_SPI
        if(spi_len >= 1)
        {
            if(!control_valid)
            {
                TOGGLE_PIN(LED_GPIO, LED_PIN);
            }
            adc_spi = spi_data[0];
            spi_valid = 1;
            spi_len = 0;
//            TRACE
//            print_hex(adc_spi);
        }
#endif

        
        PET_WATCHDOG
	}
}





