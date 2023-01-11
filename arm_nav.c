// receive navigation data from Odroid


#include "arm_truck.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_usart.h"
#include "misc.h"

#ifdef UART_NAV

static void get_code1();

static void get_payload()
{
	truck.nav.receive_buf[truck.nav.receive_size++] = truck.nav.data;
	if(truck.nav.receive_size >= 4)
	{
		truck.nav.got_data = 1;
		truck.nav.current_function = get_code1;
	}
}

static void get_code4()
{
	if(truck.nav.data == 0xe5)
	{
		truck.nav.current_function = get_payload;
		truck.nav.receive_size = 0;
	}
	else
		truck.nav.current_function = get_code1;
}

static void get_code3()
{
	if(truck.nav.data == 0xd4)
		truck.nav.current_function = get_code4;
	else
		truck.nav.current_function = get_code1;
}

static void get_code2()
{
	if(truck.nav.data == 0x2d)
		truck.nav.current_function = get_code3;
	else
		truck.nav.current_function = get_code1;
}

static void get_code1()
{
	if(truck.nav.data == 0xff)
		truck.nav.current_function = get_code2;
}


void UART5_IRQHandler(void)
{
	truck.nav.data = UART5->DR;
	truck.nav.current_function();
}



void init_nav_uart()
{
    GPIO_InitTypeDef GPIO_InitStructure;


	truck.nav.current_function = get_code1;


	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
#define UART_RX_PIN 2
#define UART_TX_PIN 12
	GPIO_PinAFConfig(GPIOD, UART_RX_PIN, GPIO_AF_UART5);
	GPIO_PinAFConfig(GPIOC, UART_TX_PIN, GPIO_AF_UART5);
	
// RX enabled
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin = 1 << UART_RX_PIN;
  	GPIO_Init(GPIOD, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = 1 << UART_TX_PIN;
  	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	USART_InitTypeDef USART_InitStructure;
// once a device is configured
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
/* USART configuration */
  	USART_Init(UART5, &USART_InitStructure);
/* Enable USART */
  	USART_Cmd(UART5, ENABLE);


/* Enable the UART Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
  	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);

	
}


#endif // UART_NAV






