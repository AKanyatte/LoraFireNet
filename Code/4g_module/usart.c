#include "usart.h"

void clockInit(void)
{
//enable HSI and wait for it to be ready

		RCC->CR |= RCC_CR_HSION;
    while (((RCC->CR) & (RCC_CR_HSION | RCC_CR_HSIRDY)) == 0);
			
//enable HSE with Bypass and wait for it to be ready

		RCC->CR |= RCC_CR_HSEON | RCC_CR_HSEBYP;
    while (((RCC->CR) & (RCC_CR_HSEON | RCC_CR_HSEBYP | RCC_CR_HSERDY)) == 0);

//SET HSE as SYSCLK and wait for it to be recognized

		RCC->CFGR = RCC_CFGR_SW_HSE;
    while (((RCC->CFGR) & (RCC_CFGR_SW_HSE | RCC_CFGR_SWS_HSE)) == 0);

//****************To Use PLL as SYSCLK	
// Disable PLL. 
		
		RCC->CR &= ~RCC_CR_PLLON;
	
		RCC->CFGR = 0x001C0000;// 36 MHz
//ENABLE PLL and wait for it to be ready

    RCC->CR |= RCC_CR_PLLON;
    while (((RCC->CR) & (RCC_CR_PLLON | RCC_CR_PLLRDY)) == 0);
   
// Set PLL as SYSCLK and wait for it to be ready
			
    RCC->CFGR |= RCC_CFGR_SW_PLL;// 0x00000002;
    while (((RCC->CFGR) & (RCC_CFGR_SW_PLL | RCC_CFGR_SWS_PLL)) == 0);
			
//Enable AFIO, Port A and USART1 clocks
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
	
}

void usart_config(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN | RCC_APB2ENR_USART1EN;
	
	GPIOA->CRH |= GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9;
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	
	GPIOA->CRH |= GPIO_CRH_CNF10_1;
	GPIOA->CRH &= ~GPIO_CRH_CNF10_0 & ~GPIO_CRH_MODE10;
	//GPIOA->CRH |= GPIO_CRH_CNF10_0;
	//GPIOA->CRH &= ~GPIO_CRH_CNF10_1 & ~GPIO_CRH_MODE10;
	
	USART1->CR1 |= USART_CR1_UE;
	USART1->BRR |= 0x138; //0xEA6; 
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;
	// 1 stop bit
	USART1->CR2 &= ~USART_CR2_STOP;
	// set word length to 8 data bits
	USART1->CR1 |= USART_CR1_M;
}

void send_data_uart(uint8_t x)
{
	while(!(USART1->SR & USART_SR_TXE))
	{
	}
	
	USART1->DR = x;
}

uint8_t receive_data_uart(void)
{
	uint8_t value;
	while(!(USART1->SR & USART_SR_RXNE))
	{
	}
	value = USART1->DR;
	return value;
}

void msg_transmit(uint8_t *pData, unsigned int Size)
{
		for(uint16_t i = 0; i < Size; i++)
		{
			send_data_uart(pData[i]);
		}
}

void msg_receive(uint8_t *pData, unsigned int Size)
{
	for (uint16_t i = 0; i < Size; i++)
	{
		pData[i] = receive_data_uart();
	} 
	/*
	uint8_t data ;
	uint8_t rx_buffer[Size];
	uint8_t rx_buffer_head = 0;
	uint8_t rx_buffer_tail = 0;
	uint16_t i = 0;
	
	while(i < Size -1)
	{
		if(rx_buffer_head != rx_buffer_tail)
		{
			data = rx_buffer[rx_buffer_tail];
			rx_buffer_tail = (rx_buffer_tail + 1) % Size;
			
			if (data == 0x0A || data == 0x0D)
				break;
			
			pData[i++] = data;
			//send_data_uart(data) Echo data
		}
	}
	pData[i] = 0x00;*/
	/*
	for (uint16_t i = 0; i < Size; i++)
	{
		pData[i] = receive_data_uart();
	} */    
}

void delay(uint32_t time_delay)
{
	while(time_delay--)
	{
	}
}

void led_init(void)
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	GPIOA->CRL |= GPIO_CRL_MODE5;
  GPIOA->CRL &= ~GPIO_CRL_CNF5;
}

void led_on(void)
{
	 GPIOA->ODR |= GPIO_ODR_ODR5;
}

void led_off(void)
{
	GPIOA->ODR &= ~GPIO_ODR_ODR5;
}

