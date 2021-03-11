#include "arm_xbee.h"


#ifdef USE_XBEE

#include "linux.h"
#include "misc.h"
#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "uart.h"

radio_t radio;
static USART_InitTypeDef USART_InitStructure;

static void get_code1();

#ifdef USE_HOPPING

// Must enter API mode by plugging the radio into a computer at 9600 baud
// sleep 1 second
// send +++             page 24
// sleep 1 second
// send ATAP1     enable API mode page 35
// send atbd7     115200 baud  page 36
// send ATWR        write to flash
// send ATCN        exit command mode



// frequency hopping table
// channels are on page 37
const uint8_t freqs[] = 
{
    12,
    23,
    17,
    14,
    20,
    16,
    22,
    13,
    18,
    21,
    15,
    19
};



#define TOTAL_FREQS sizeof(freqs)
// timer CNT's between hops
#define LOCKED_DELAY -10
#define UNLOCKED_DELAY -20
// wait this long after getting a packet to hop
#define HOP_LAG -5
// ticks before becoming unlocked
#define LOCKED_TIMEOUT 1000


void send_radio(unsigned char *ptr, int size)
{
    radio.buffer = ptr;
    radio.buffer_ptr = 0;
    radio.buffer_size = size;
    
    USART1->DR = radio.buffer[0];
    radio.buffer_ptr++;
}

static void set_freq(int index)
{
    uint8_t freq = freqs[index % TOTAL_FREQS];

    packet[0] = 0x7e;    // sync code
    packet[1] = 0x00;    // size
    packet[2] = 0x06;    // size

    packet[3] = 0x08;    // AT command
    packet[4] = 0x00;    // no ACK
    packet[5] = 'C';
    packet[6] = 'H';
    packet[7] = hex_chars[(uint8_t)((freq >> 4) & 0x0f)];
    packet[8] = hex_chars[(uint8_t)(freq & 0x0f)];

    packet[9] = chksum(packet + 3, 6);

    write_uart(packet, 10);
}


// time to hop frequencies
void TIM4_IRQHandler()
{
	if(TIM4->SR & TIM_FLAG_Update)
	{
		TIM4->SR = ~TIM_FLAG_Update;
        
        
        if(radio.initialized)
        {
// hop
            radio.timer_freq++;
            set_freq(radio.timer_freq);

// schedule locked delay
            if(radio.locked_in)
            {
                TIM4->CNT = LOCKED_DELAY;
            }
            else
// schedule unlocked delay
            {
                TIM4->CNT = UNLOCKED_DELAY;
            }
        }
    }
}


void get_chksum()
{
    radio.current_function = get_code1;

// is it our packet?
    if(radio.packet[0] == 0x2d &&
        radio.packet[1] == 0xd4)
    {
// send it to the truck
        


// resync the hopping timer to the packets
        TIM4->CNT = HOP_LAG;
        radio.locked_in = 1;
// reset the locked timeout
        radio.start_time = truck.tick;
    }
}


void get_packet()
{
    if(radio.packet_offset < PACKET_SIZE)
    {
        radio.packet[radio.packet_offset++] = radio.data;
    }
    
    radio.length--;
    
    if(radio.length == 0)
    {
        radio.current_function = get_chksum;
    }
}

void get_options()
{
    radio.length--;
    radio.packet_offset = 0;
    radio.current_function = get_packet;
}


void get_rssi()
{
    radio.length--;
    radio.current_function = get_options;
}


void get_address2()
{
    if(radio.data == 0xff)
    {
        radio.length--;
        radio.current_function = get_rssi;
    }
    else
    {
        radio.current_function = get_code1;
    }
}


void get_address1()
{
    if(radio.data == 0xff)
    {
        radio.length--;
        radio.current_function = get_address2;
    }
    else
    {
        radio.current_function = get_code1;
    }
}


void get_api_id()
{
    if(radio.data == 0x81)
    {
        // API identifier
        radio.length--;
        radio.current_function = get_address1;
    }
    else
    {
        radio.current_function = get_code1;
    }
}

void get_length2()
{
    radio.length |= radio.data;
    radio.current_function = get_api_id;
}

void get_length1()
{
    radio.length = radio.data << 8;
    radio.current_function = get_length2;
}

void get_code1()
{
    if(radio.data == 0x7e)
    {
        radio.current_function = get_length1;
    }
}



void config3()
{
    if(truck.tick - radio.start_time > TIMER_HZ)
    {
// reconfigure it
	    USART_InitStructure.USART_BaudRate = 115200;
  	    USART_Init(USART1, &USART_InitStructure);

// the starting frequency
        set_freq(0);

// start parsing serial input in the API format. page 56
        radio.current_function = get_code1;
        radio.initialized = 1;
    }
}

void config2()
{
    if(truck.tick - radio.start_time > TIMER_HZ)
    {
// enter API mode.  page 35
// Set 115200 baud. page 36
        send_radio("ATAP1\r\nATBD7\r\nCN\r\n", 18);
        radio.start_time = truck.tick;
        radio.current_function = config3;
    }
}

void config1()
{
    if(truck.tick - radio.start_time > TIMER_HZ)
    {
        send_radio("+++", 3);
        radio.start_time = truck.tick;
        radio.current_function = config2;
    }
}





void USART1_IRQHandler(void)
{
// got data from the radio
    if(USART1->SR & USART_FLAG_RXNE)
    {
        if(radio.initialized)
        {
    	    radio.data = USART1->DR;
	        radio.current_function();
        }
    }

// send data to the radio
    if((USART1->SR & USART_FLAG_TXE))
    {
        USART1->SR = (uint16_t)~USART_FLAG_TXE;
        if(radio.buffer_ptr < radio.buffer_size)
        {
            USART1->DR = radio.buffer[radio.buffer_ptr];
            radio.buffer_ptr++;
        }
    }

}

#else // USE_HOPPING


static void get_data()
{
	radio.next_packet[radio.counter++] = radio.data;
	if(radio.counter >= PACKET_SIZE)
	{
		radio.current_function = get_code1;
		if(!radio.got_packet)
		{
			radio.got_packet = 1;
			memcpy(radio.packet, radio.next_packet, PACKET_SIZE);
		}
	}
}

static void get_code3()
{
	if(radio.data == 0xd4)
	{
		radio.current_function = get_data;
		radio.counter = 0;
	}
	else
	{
		radio.current_function = get_code1;
	}
}

static void get_code2()
{
	if(radio.data == 0x2d)
	{
		radio.current_function = get_code3;
	}
	else
	{
		radio.current_function = get_code1;
	}
}

static void get_code1()
{
	if(radio.data == 0xff)
	{
		radio.current_function = get_code2;
	}
}


void USART1_IRQHandler(void)
{
// got data from the radio
    if(USART1->SR & USART_FLAG_RXNE)
    {
    	radio.data = USART1->DR;
	    radio.current_function();
    }

}

#endif // !USE_HOPPING





void init_xbee()
{
	bzero(&radio, sizeof(radio_t));


	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;


	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
#define UART_RX_PIN 10
	GPIO_PinAFConfig(GPIOA, UART_RX_PIN, GPIO_AF_USART1);
    
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = (1 << UART_RX_PIN);
  	GPIO_Init(GPIOA, &GPIO_InitStructure);

// Original xbee
//	USART_InitStructure.USART_BaudRate = 9600;
// 3DRRadio/XBee 900
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* USART configuration */
  	USART_Init(USART1, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(USART1, ENABLE);

/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);



#ifdef USE_HOPPING
#define UART_TX_PIN 9
	GPIO_PinAFConfig(GPIOA, UART_TX_PIN, GPIO_AF_USART1);
	GPIO_InitStructure.GPIO_Pin = (1 << UART_TX_PIN);
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
  	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);




// this timer causes the frequency change operations
// timer wraps at 168000000 / (TIM_Prescaler + 1) / (TIM_Period + 1)
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 65535;
// each increment is 1ms
//	TIM_TimeBaseStructure.TIM_Prescaler = 999;
// DEBUG each increment is 10ms
	TIM_TimeBaseStructure.TIM_Prescaler = 9999;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM4, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
    
    radio.start_time = truck.tick;
    radio.current_function = config1;
#else // USE_HOPPING

	radio.current_function = get_code1;

#endif // !USE_HOPPING







    
}





#endif // USE_XBEE

