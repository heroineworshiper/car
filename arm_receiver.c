/*
 * HIGH PERFORMANCE RECEIVER
 * Copyright (C) 2026 Adam Williams <broadcast at earthling dot net>
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

// make receiver.bin;./uart_programmer receiver.bin


#include "linux.h"
#include "uart.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_iwdg.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_syscfg.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_spi.h"

#define RADIO_CS_GPIO GPIOB
#define RADIO_CS_PIN GPIO_Pin_12
#define RADIO_SDO_GPIO GPIOB
#define RADIO_SDO_PIN GPIO_Pin_15
#define RADIO_CLK_GPIO GPIOB
#define RADIO_CLK_PIN GPIO_Pin_13

#define LED_PIN GPIO_Pin_13
#define LED_GPIO GPIOC

#define HZ 1000
//#define SPI_SPEED 1000000  // 125kbit
#define SPI_SPEED 800000  // 100kbit
//#define SPI_SPEED 400000  // 50kbit
#define SPI_BUFSIZE 2048
#define DMA_STREAM DMA1_Stream3
uint8_t dma_buffer[SPI_BUFSIZE];
// read position
int dma_pointer = 0;
// number of 1 bits in a byte
uint8_t bitcount[256];
uint16_t sample_buffer;
int current_level = 0;
int level_counter = 0;
volatile int tick = 0;
uint8_t output_buffer[5];
int output_size = 0;
int output_ptr = 0;
RCC_ClocksTypeDef RCC_ClocksStatus;

const uint8_t PACKET_KEY[] = 
{
    0x5b, 0xb1, 0x6e, 0x6b, 0x33, 0x30, 0x9e, 0x08
};

const uint8_t DATA_KEY[] =
{
    0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xaa, 0xaa, 0xaa, 0x55, 0x55, 0x55
};
#define PACKET_DATA 12


#define RADIO_BUFSIZE (sizeof(PACKET_KEY) + PACKET_DATA)
volatile unsigned char receive_buf[RADIO_BUFSIZE];

// frequency hopping table.  Use freqs433.ods to calculate.
const uint16_t channels[] = 
{
//    1200, // 433Mhz/desk transmitter
//    1300, // 433.25
    1400, // 433.5Mhz
//    1500, // 433.75Mhz
//    1600, // 434Mhz
//    1700, // 434.25Mhz
//    1800  // 434.5Mhz
};

// scan for synchronous code
#define FIFORSTREG 0xCA81
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


// AFC command. page 22
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


// Receiver control command.  page 19
// Page 37 of the SI4421 datasheet gives optimum bandwidth values
// Best results observed at 400khz  Applied regardless of BBFCREG value.
//#define RXCREG 0x9480     // BW 200KHz, LNA gain 0dB, RSSI -103dBm
//#define RXCREG 0x9440     // BW 340KHz, LNA gain 0dB, RSSI -103dBm
#define RXCREG 0x9420       // BW 400KHz, LNA gain 0dB, RSSI -103dBm

//#define TXCREG 0x9850     // FSK shift: 90kHz
#define TXCREG 0x98f0       // FSK shift: 165kHz
#define STSREG 0x0000
#define RXFIFOREG 0xb000

// Baseband filter type.  page 20
#define BBFCREG                 0xc23c // analog filter
//#define BBFCREG                 0xc22c // digital filter

// DEBUG uart
void USART6_IRQHandler(void)
{
	unsigned char c = USART6->DR;
	uart.input = c;
	uart.got_input = 1;
}

void TIM1_UP_TIM10_IRQHandler()
{
	if(TIM10->SR & TIM_FLAG_Update)
	{
		TIM10->SR = ~TIM_FLAG_Update;
		
		tick++;
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
    GPIO_InitTypeDef GPIO_InitStructure;

// initialize radio SPI
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Pin = RADIO_CS_PIN |
        RADIO_SDO_PIN |
        RADIO_CLK_PIN;
    GPIO_Init(RADIO_CS_GPIO, &GPIO_InitStructure);
    SET_PIN(RADIO_CS_GPIO, RADIO_CS_PIN);
    CLEAR_PIN(RADIO_SDO_GPIO, RADIO_SDO_PIN);
    CLEAR_PIN(RADIO_CLK_GPIO, RADIO_CLK_PIN);
    
    

// disable radio for testing
#if 1
// scan for synchronous code
    write_radio(FIFORSTREG);
// enable synchron latch
    write_radio(FIFORSTREG | 0x0002);
    write_radio(GENCREG);
    write_radio(AFCCREG);
    write_radio(CFSREG(channels[0]));
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
#endif // 0


// initialize the bitcount table
    int i;
    for(i = 0; i < 256; i++)
    {
        int j;
        int count = 0;
        for(j = 0; j < 8; j++)
            if((i & (1 << j))) count++;
        bitcount[i] = count;
    }

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
// data
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
// clock
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


    SPI_InitTypeDef SPI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    DMA_StructInit(&DMA_InitStructure);
    SPI_StructInit(&SPI_InitStructure);
// select channel, stream, DMA# on page 164
  	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
  	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dma_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = SPI_BUFSIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
// more stable capture if FIFO disabled
  	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


	SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

// generate SPI clock from timer to precisely control it
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_Period = RCC_ClocksStatus.SYSCLK_Frequency / 2 / 
        SPI_SPEED - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_Pulse = 0;
 	TIM_OC4Init(TIM3, &TIM_OCInitStructure);
    TIM3->CCR4 = TIM_TimeBaseStructure.TIM_Period / 2;
	TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM3, ENABLE);

    SPI_I2S_DeInit(SPI2);
    SPI_Init(SPI2, &SPI_InitStructure);

// page 164
	DMA_DeInit(DMA_STREAM);
  	DMA_Init(DMA_STREAM, &DMA_InitStructure);
	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, ENABLE);
	SPI_Cmd(SPI2, ENABLE);
	DMA_Cmd(DMA_STREAM, ENABLE);


}

void process_packet()
{
    int i;
    int failed = 0;

// XOR the data key
    for(i = 0; i < PACKET_DATA; i++)
    {
        receive_buf[sizeof(PACKET_KEY) + i] ^= DATA_KEY[i];
    }

    for(i = 3; i < PACKET_DATA; i += 3)
    {
        if(receive_buf[sizeof(PACKET_KEY) + 0] != receive_buf[sizeof(PACKET_KEY) + i] ||
            receive_buf[sizeof(PACKET_KEY) + 1] != receive_buf[sizeof(PACKET_KEY) + i + 1] ||
            receive_buf[sizeof(PACKET_KEY) + 2] != receive_buf[sizeof(PACKET_KEY) + i + 2])
        {
// reject packet if any value is different
            failed = 1;
            break;
        }
    }

// valid packet
    if(!failed)
    {
// cue the bottom half to send values to the host
        output_size = 5;
        output_ptr = 0;
        output_buffer[0] = 0xff;
        output_buffer[1] = 0xe7;
        output_buffer[2] = receive_buf[sizeof(PACKET_KEY) + 0];
        output_buffer[3] = receive_buf[sizeof(PACKET_KEY) + 1];
        output_buffer[4] = receive_buf[sizeof(PACKET_KEY) + 2];
        print_hex2(output_buffer[2]);
        print_text(" ");
        print_hex2(output_buffer[3]);
        print_text(" ");
        print_hex2(output_buffer[4]);
        print_lf();
    }
}


void handle_radio()
{
// load 8 samples in the newest slot
    sample_buffer |= dma_buffer[dma_pointer++];
    if(dma_pointer >= SPI_BUFSIZE) dma_pointer = 0;

    int i;
    for(i = 0; i < 8; i++)
    {
        sample_buffer <<= 1;
        int ones = bitcount[((uint8_t*)&sample_buffer)[1]];
        int got_transition = 0;
        if(level_counter > 4 && current_level && ones < 4)
        {
            got_transition = 1;
            current_level = 0;
        }
        else
        if(level_counter > 4 && !current_level && ones > 4)
        {
            got_transition = 1;
            current_level = 1;
        }
        else
            level_counter++;

        if(got_transition)
        {
            int got_value = 0;
// got a 1 bit
            if(level_counter >= 10)
                got_value = 1;
            else
// got a 0 bit
                got_value = 0;

            level_counter = 0;
            int j;
            for(j = 0; j < RADIO_BUFSIZE - 1; j++)
            {
                receive_buf[j] <<= 1;
                receive_buf[j] |= (receive_buf[j + 1] >> 7);
            }
            receive_buf[RADIO_BUFSIZE - 1] <<= 1;
            receive_buf[RADIO_BUFSIZE - 1] |= got_value;

            for(j = 0; j < sizeof(PACKET_KEY); j++)
            {
                if(receive_buf[j] != PACKET_KEY[j]) break;
            }
            if(j >= sizeof(PACKET_KEY))
            {
                process_packet();
//                print_text("got key\n");
            }
        }
    }
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


int main(void)
{
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
	print_text("Welcome to car receiver\n");
 	flush_uart();
 	print_text("SYSCLK_Frequency=");
 	print_number(RCC_ClocksStatus.SYSCLK_Frequency);
 	print_lf();
	flush_uart();
    init_watchdog();

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


// output UART
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 100000;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
/* USART configuration */
  	USART_Init(USART2, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART2, ENABLE);




// LED pin
	GPIO_InitStructure.GPIO_Pin = LED_PIN;
	GPIO_Init(LED_GPIO, &GPIO_InitStructure);
	SET_PIN(LED_GPIO, LED_PIN);
    init_radio();

    int prev_frame = 0;
    int debug_count = 0;
#define DEBUG_INTERVAL 100
	while(1)
	{
        HANDLE_UART_OUT
        
	    if((USART2->SR & USART_FLAG_TC) != 0 &&
		    output_ptr < output_size)
	    {
		    USART2->DR = output_buffer[output_ptr++];
	    }

// NDTR goes from 1-SPI_BUFSIZE
        int ndtr_value = SPI_BUFSIZE - DMA_STREAM->NDTR;
        if(ndtr_value != dma_pointer)
            handle_radio();
        PET_WATCHDOG
    }
}









