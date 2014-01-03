/*!
  * @file    arch/main.c
  * @brief   string Control API
  */
#include <stdio.h>
#include "stm32f10x.h"
#include "Lb_Printf.h"

void __attribute__((weak)) delay_ms(unsigned int msec)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)msec*(SystemCoreClock/8/1000);
	SysTick->VAL =0x00;		// clear Count flag
	SysTick->CTRL=0x01;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	// wait Count flag set
	SysTick->CTRL=0x00;
	SysTick->VAL =0X00;
}

void __attribute__((weak)) delay_us(unsigned int usec)
{
	uint32_t temp;
	SysTick->LOAD=(uint32_t)usec*(SystemCoreClock/8/1000000);
	SysTick->VAL =0x00;		// clear Count flag
	SysTick->CTRL=0x01;
	do
	{
		temp=SysTick->CTRL;
	}
	while((temp&0x01)&&!(temp&(1<<16)));	// wait Count flag set
	SysTick->CTRL=0x00;
	SysTick->VAL =0X00;
}

void print_byte(unsigned short  c)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE)==RESET);
	USART_SendData(USART1, c);
}

int GetKey(char *pkey)
{
	int ret = 0;

	if ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	{
		*pkey = (u8)USART_ReceiveData(USART1);
		ret = 1;
	}
	return ret;
}

void UART_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	// RCC Configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE );

	// Port 설정
	/* J7-11 PA9 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* J7-12 PA10 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// UART Port 설정
	USART_InitStructure.USART_BaudRate    = 115200  ;
	USART_InitStructure.USART_WordLength   = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits    = USART_StopBits_1;
	USART_InitStructure.USART_Parity    = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode    = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);
	USART_Cmd(USART1, ENABLE);
}


int cap_rising_edge = 0;
int cap_falling_edge = 0;
int pulse_width = 0;
int bCapture = 0;
int bDistance = 0;

void init_hrsd04_variable()
{
	cap_rising_edge = 0;
	cap_falling_edge = 0;
	pulse_width = 0;
	bCapture = 0;
	bDistance = 0;
}

void Triger_InputSig(void)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, 1);
	delay_us(10);
	GPIO_WriteBit(GPIOA, GPIO_Pin_7, 0);
}

void TIM3_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM3, TIM_IT_CC1)==SET )
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);

		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_6)==Bit_SET)	// Timer3 Ch1 pin(PA6) is High
		{
			cap_rising_edge = TIM_GetCapture1(TIM3);	// read capture data
			TIM3->CCER |=  TIM_CCER_CC1P;	// to falling edge
		}
		else			// Timer3 Ch1 pin(PA6) is Low
		{
			cap_falling_edge = TIM_GetCapture1(TIM3);	// read capture data
			pulse_width = (u32) (cap_falling_edge - cap_rising_edge);
			TIM3->CCER &= ~ TIM_CCER_CC1P;	// to rising edge

			bDistance = 1;
		}
	}

}

void setup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;

	// RCC Configuration
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );

	// GPIO Output for Trigger
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// TIM3 Ch1 (PA6)
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// RCC Configuration
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE );

	// Enable the TIM3 global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	// 1usec for 72MHz clock
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel3 */
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	TIM_Cmd(TIM3, ENABLE);

	TIM_ITConfig(TIM3, TIM_IT_CC1, ENABLE);

	init_hrsd04_variable();

	Lb_printf("Input Capture with TIM3\r\n");
}

void loop(void)
{
	char c;

	if(GetKey(&c)==1)
	{
		switch(c)
		{
		case '1':
			bCapture = 1;
			Triger_InputSig();
			break;

		case '2':
			bCapture = 0;
			init_hrsd04_variable();
			break;

		default:
			Lb_printf("%c", c);
			if(c=='\r')
				Lb_printf("\n");
			break;
		}
	}

	if(bDistance!=0)
	{
		bDistance = 0;
		Lb_printf("(%d usec) %5d mm \r\n", pulse_width, pulse_width*17/100);
		if(bCapture)
		{
			Triger_InputSig();
		}
	}

	delay_ms(500);
}

int main(void)
{
	UART_Configuration();

	setup();

	while(1)
	{
		loop();
	}
}
