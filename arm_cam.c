/*
 * STM32 CONTROLLER FOR CAMERA PANNER
 * Copyright (C) 2020-2024 Adam Williams <broadcast at earthling dot net>
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
// USART1 is the servo at 115kbaud
// USART3 is the radio at 100kbaud
// USART6 is the debug

// send a motor command to the ttyACM device:
// echo -n -e '\xff\xd2\x80' > /dev/ttyACM0
// echo -n -e '\xff\xd2\x00' > /dev/ttyACM0
// echo -n -e '\xff\xd2\xff' > /dev/ttyACM0


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
#include "arm_fs.h"
#include "arm_usb.h"
#include "usb_core.h"

// settings
// time
#define HZ 1000

// use A14 CLK & A13 DATA for external SPI commands
//#define USE_SPI
#define USE_USB

// reverse direction
#define SIGN -1

// time before shutting motor down
#define MOTOR_TIMEOUT (HZ / 4)



// analog ranges for the remote transmitter
#define ADC_CENTER 128
#define ADC_DEADBAND 5
#define ADC_MAX 64
// minimums
#define MIN_LEFT (ADC_CENTER + ADC_DEADBAND)
#define MIN_RIGHT (ADC_CENTER - ADC_DEADBAND)
// maximums
#define MAX_LEFT (ADC_CENTER + ADC_MAX)
#define MAX_RIGHT (ADC_CENTER - ADC_MAX)

#define FRACTION 256

#define MAX_STEP (24 * FRACTION)
#define MIN_STEP (FRACTION)
#define ACCELERATION (8 * FRACTION / 256)

//#define DEBUG_PIN GPIO_Pin_4
//#define DEBUG_GPIO GPIOB

#define LED_PIN GPIO_Pin_13
#define LED_GPIO GPIOC

#define BRAKE_PIN GPIO_Pin_14
#define BRAKE_GPIO GPIOA

// SPI
#define CLK_PIN GPIO_Pin_14
#define DAT_PIN GPIO_Pin_13
#define SPI_GPIO GPIOA

// idle ticks before resetting SPI
#define SPI_TICKS 10
// ticks before timing out the SPI/USB code
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

// frequency hopping table.  Use freqs433.ods to calculate.
const uint16_t channels[] = 
{
//    1200, // 433Mhz/desk transmitter
//    1300, // 433.25
//    1400, // 433.5Mhz
//    1500, // 433.75Mhz
//    1600, // 434Mhz
//    1700, // 434.25Mhz
    1800  // 434.5Mhz
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

// 915MHz page 16
//#define FREQ_BAND 0x0030
// 433Mhz
#define FREQ_BAND 0x0010

// frequency page 18
#define CFSREG(chan) (0xA000 | (chan))
// crystal load 10pF page 16
#define XTAL_LD_CAP 0x0003
#define GENCREG (0x8000 | XTAL_LD_CAP | FREQ_BAND)
// power management page 16
#define PMCREG 0x8201


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
// time of last hop
volatile int last_hop = 0;
volatile int need_hop = 0;
volatile int scanning = 0;
#define MAX_MISSED_PACKETS HOP_HZ





#define ABS(x) ((x) > 0 ? (x) : (-(x)))

// 0 - SIN_TOTAL * FRACTION
int phase = 0;
// phase change per frame * FRACTION
int step = 0;
int goal_step = 0;
// motion control state
int motion_control_on = 0;
// a timelapse code indicating the last direction
int motion_control_dir = 0xff;
int motion_control_tick = 0;

// configuration
// configured for single camera movement instead of timelapse
volatile int motion_control = 1;
// speed of the movement
// 1-80
volatile int motion_control_speed = 60;
// ticks of the movement duration
volatile int motion_control_len = HZ;
// timelapse speed 1-80
// 1 = 20x timelapse speed
volatile int timelapse_speed = 1;

// motor codes
#define GROUND_MOTOR 0xfe
#define COAST_MOTOR 0xff
#define SIN_TOTAL 252
int motor_enabled = 0;

#define MAX_POWER 0xc0
#define MIN_POWER 0x20
int motor_power = MAX_POWER;

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
// ticks before timing out the RF code
#define RADIO_TIMEOUT HZ
volatile int timeout_counter = 0;


// values from the radio
volatile uint8_t adc_raw = 0xff;
volatile uint8_t timelapse_code = 0xff;
#define TIMELAPSE_LEFT 0
#define TIMELAPSE_RIGHT 1
volatile uint8_t rf_valid = 0;
volatile uint8_t spi_valid = 0;
RCC_ClocksTypeDef RCC_ClocksStatus;

#define RADIO_CS_GPIO GPIOB
#define RADIO_CS_PIN GPIO_Pin_12
#define RADIO_SDO_GPIO GPIOB
#define RADIO_SDO_PIN GPIO_Pin_14
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
        if(PIN_IS_SET(BRAKE_GPIO, BRAKE_PIN))
            motor_packet[3] = COAST_MOTOR;
        else
            motor_packet[3] = GROUND_MOTOR;
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

// flash for RF reception
            TOGGLE_PIN(LED_GPIO, LED_PIN);
            timeout_counter = 0;

            uint8_t code_byte = receive_buf[0];
            timelapse_code = (code_byte & 0xf);
            current_channel = (code_byte >> 4);
            adc_raw = receive_buf[1];
            rf_valid = 1;
            
            int time_diff = packet_tick - last_hop;


// TRACE2
print_text("chan=");
print_number(current_channel);
print_text("timelapse_code=");
print_number(timelapse_code);
print_text("adc_raw=");
print_number(adc_raw);
//print_text("GOT IT ");
// hop lag observed
print_text("hop lag=");
print_number(time_diff);
print_lf();

// schedule the next hop based on the end time of this packet
            scanning = 0;
            missed_packets = 0;
            next_hop = packet_tick + HZ / HOP_HZ - HOP_LAG;
        }
    }
}

void get_key()
{
//TOGGLE_PIN(LED_GPIO, LED_PIN);
//send_uart(radio_data);
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
// need the delay if the system clock is over 16Mhz
    udelay(1);

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
        udelay(1);
        SET_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
        udelay(1);
        CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    }
    
    udelay(1);
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
// cam transmitter
	USART_InitStructure.USART_BaudRate = 100000;
// desk transmitter
//	USART_InitStructure.USART_BaudRate = 4000;

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









void init_motor()
{
// transmit pin for motor UART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
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
  	USART_Init(USART1, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART1, ENABLE);


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
		
		tick++;


// reset the control code
        if(timeout_counter >= RADIO_TIMEOUT)
        {
// solid for power on
            if(rf_valid) SET_PIN(LED_GPIO, LED_PIN);
// emergency shut down of the motor
            rf_valid = 0;
            motion_control_on = 0;
        }
        else
        {
            timeout_counter++;
        }


        if(tick >= next_hop)
        {
            last_hop = tick;
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


void dump_config()
{
	TRACE2
	print_text("\nmotion_control=");
	print_number(motion_control);
	print_text("\nmotion_control_speed=");
	print_number(motion_control_speed);
	print_text("\nmotion_control_len=");
	print_number(motion_control_len);
	print_text("\ntimelapse_speed=");
	print_number(timelapse_speed);
    print_lf();
    flush_uart();
}

#ifdef USE_USB
extern unsigned char *in_buf;
static void handle_code1(uint8_t c);
void (*code_state)(uint8_t c) = handle_code1;
int config_offset = 0;
// Magic number for settings
#define CONFIG_MAGIC 0x10291976
#define CONFIG_SIZE 7
#define CONFIG_SIZE_ROUNDED 8
// must write a multiple of 4 bytes
uint8_t config_packet[CONFIG_SIZE_ROUNDED] = { 0 };

void flush_motor()
{
    while(motor_offset < MOTOR_PACKET)
    {
	    if((USART1->SR & USART_FLAG_TC) != 0)
        {
            USART1->DR = motor_packet[motor_offset++];
        }
    }
}

void write_config()
{
// must write a multiple of 4 bytes
    save_file(CONFIG_MAGIC,
        config_packet,
		CONFIG_SIZE_ROUNDED);


// wiggle motor
    int i;
    int steps = 200;
    flush_motor();
    motor_power = MAX_POWER;
    motor_enabled = 1;
    for(i = 0; i < steps; i++)
    {
        if(i < steps / 2)
            phase += MAX_STEP;
        else
            phase -= MAX_STEP;
        while(phase < 0)
            phase += SIN_TOTAL * FRACTION;
        while(phase >= SIN_TOTAL * FRACTION)
            phase -= SIN_TOTAL * FRACTION;
        write_motor();
        flush_motor();
        udelay(5000);
        PET_WATCHDOG
    }

    motor_power = MIN_POWER;
    motor_enabled = 0;
    write_motor();
}

void read_config()
{
// Load settings from flash
	uint32_t address = next_address(CONFIG_MAGIC);
	if(address != 0xffffffff)
    {
        address += 8;
        const uint8_t *buffer = (unsigned char*)address;
        const uint8_t *ptr = buffer;
		motion_control = *ptr++;
        motion_control_speed = *ptr++;
        motion_control_len = *ptr++;
        motion_control_len |= ((uint32_t)(*ptr++)) << 8;
        motion_control_len |= ((uint32_t)(*ptr++)) << 16;
        motion_control_len |= ((uint32_t)(*ptr++)) << 24;
        timelapse_speed = *ptr++;
	}
    else
    {
        TRACE2
        print_text("config not found\n");
    }
}


static void handle_config_code(uint8_t c)
{
    config_packet[config_offset++] = c;
    if(config_offset >= CONFIG_SIZE)
    {
        write_config();
        read_config();
        dump_config();
// reset it
        motion_control_dir = 0xff;
        motion_control_on = 0;
        code_state = handle_code1;
    }
}

static void handle_adc_code(uint8_t c)
{
    adc_usb = c;
    print_text("ADC=");
    print_hex(adc_usb);
    print_lf();

    if(!rf_valid)
    {
// flash for USB reception if not receiving RF
        TOGGLE_PIN(LED_GPIO, LED_PIN);
    }
    last_usb_tick = tick;
    usb_valid = 1;
    code_state = handle_code1;
}

static void handle_code2(uint8_t c)
{
    if(c == 0xd2)
        code_state = handle_adc_code;
    else
    if(c == 0xcf)
    {
        code_state = handle_config_code;
        config_offset = 0;
    }
    else
    if(c != 0xff)
        code_state = handle_code1;
}

static void handle_code1(uint8_t c)
{
    if(c == 0xff)
        code_state = handle_code2;
}

// got chars from the host
uint8_t class_cb_DataOut (void  *pdev, uint8_t epnum)
{
    int USB_Rx_Cnt = ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;


// process the codes
    int i;
    for(i = 0; i < USB_Rx_Cnt; i++)
    {
        code_state(in_buf[i]);
    }

    usb_start_receive();
    return 0;
}
#endif // USE_USB



int main(void)
{
	int i, j;
 
 
 
// switch system clock to non PLL
//    RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
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
//    SystemCoreClockUpdate();

	init_linux();

/* Enable the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA |
			RCC_AHB1Periph_GPIOB |
            RCC_AHB1Periph_GPIOC, 
		ENABLE);

// enable the interrupt handler
 	NVIC_SetVectorTable(NVIC_VectTab_FLASH, PROGRAM_START - 0x08000000);

	init_uart();


	RCC_GetClocksFreq(&RCC_ClocksStatus);
	print_text("Welcome to cam panner\n");
// 	flush_uart();
// 	print_text("SYSCLK_Frequency=");
// 	print_number(RCC_ClocksStatus.SYSCLK_Frequency);
// 	print_lf();
// 	print_text("SystemCoreClock=");
// 	print_number(SystemCoreClock);
// 	print_lf();
// 	print_text("HCLK_Frequency=");
// 	print_number(RCC_ClocksStatus.HCLK_Frequency);
// 	print_lf();
// 	print_text("PCLK1_Frequency=");
// 	print_number(RCC_ClocksStatus.PCLK1_Frequency);
// 	print_lf();
// 	print_text("PCLK2_Frequency=");
// 	print_number(RCC_ClocksStatus.PCLK2_Frequency);
// 	print_lf();

	flush_uart();
    init_watchdog();
	flush_uart();
    list_flash();

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
#endif // USE_SPI

// init the brake button
    GPIO_InitStructure.GPIO_Pin = BRAKE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(BRAKE_GPIO, &GPIO_InitStructure);



	init_motor();
    init_radio();

// wait a while
//     while(tick < HZ)
//     {
//         ;
//     }

#ifdef USE_USB
	init_usb();
    read_config();
#endif

    dump_config();

    int seconds = 0;
    int prev_frame = 0;
    int debug_count = 0;
	while(1)
	{
        HANDLE_UART_OUT

// transmit motor code
	    if((USART1->SR & USART_FLAG_TC) != 0)
        {
            if(motor_offset < MOTOR_PACKET)
            {
                USART1->DR = motor_packet[motor_offset++];
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
            if(!rf_valid && 
                tick - last_spi_tick < SPI_TIMEOUT)
            {
                adc = adc_spi;
            }
            else
            {
// solid for power on
                if(!rf_valid) SET_PIN(LED_GPIO, LED_PIN);
                spi_valid  = 0;
            }
#endif

#ifdef USE_USB
            if(!rf_valid &&
                tick - last_usb_tick < SPI_TIMEOUT)
            {
                adc = adc_usb;
            }
            else
            {
// solid for power on
                if(!rf_valid) SET_PIN(LED_GPIO, LED_PIN);
                usb_valid = 0;
            }
#endif


            goal_step = 0;
            if(rf_valid || spi_valid || usb_valid)
            {
// exit motion control if out of timelapse mode
                if((motion_control_on || motion_control_dir < 2) &&
                    timelapse_code >= 2)
                {
// reset the motion control state
                    motion_control_on = 0;
                    motion_control_dir = 0xff;
                    goal_step = 0;
                }

                if(motion_control && rf_valid && timelapse_code < 2)
                {
// start motion control
                    if(adc <= MAX_RIGHT && 
                        motion_control_dir != TIMELAPSE_RIGHT)
                    {
                        motion_control_on = 1;
                        motion_control_dir = TIMELAPSE_RIGHT;
                        motion_control_tick = 0;
                    }
                    else
                    if(adc >= MAX_LEFT && 
                        motion_control_dir != TIMELAPSE_LEFT)
                    {
                        motion_control_on = 1;
                        motion_control_dir = TIMELAPSE_LEFT;
                        motion_control_tick = 0;
                    }
                }
                else
// manual or timelapse mode
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
                if(rf_valid && timelapse_code < 2)
                {
// handle the timelapse code
                    if(timelapse_code == TIMELAPSE_LEFT)
                        goal_step = MAX_STEP * timelapse_speed / 80;
                    else
                    if(timelapse_code == TIMELAPSE_RIGHT)
                        goal_step = -MAX_STEP * timelapse_speed / 80;
                }
            }

// update motion control
            if(motion_control_on)
            {
                if(motion_control_dir == TIMELAPSE_LEFT)
                    goal_step = MAX_STEP * motion_control_speed / 80;
                else
                    goal_step = -MAX_STEP * motion_control_speed / 80;
            
                if(motion_control_tick >= motion_control_len)
                {
                    motion_control_on = 0;
                    goal_step = 0;
                }
                else
                    motion_control_tick++;
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
                phase += SIGN * step;
                while(phase < 0)
                    phase += SIN_TOTAL * FRACTION;
                while(phase >= SIN_TOTAL * FRACTION)
                    phase -= SIN_TOTAL * FRACTION;
            }

            write_motor();



//             if(debug_count == 0)
//             {
//                  print_text("adc=");
//                  print_number(adc);
//                  print_text("timelapse=");
//                  print_number(timelapse_code);
//                  print_text("step=");
//                  print_fixed(step);
//                  print_text("power=");
//                  print_number(motor_power);
//                  print_text("enabled=");
//                  print_number(motor_enabled);
//                  print_lf();
//             }

        }


// check clock
        if(tick / HZ != seconds)
        {
            seconds = tick / HZ;
//             print_text("seconds=");
//             print_number(seconds);
//             print_lf();
        }




#ifdef USE_SPI
        if(spi_len >= 1)
        {
            if(!rf_valid)
            {
// flash for SPI reception
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





