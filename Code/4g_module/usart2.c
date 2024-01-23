#include "usart.h"
#include "usart2.h"

void usart2_config(void)
{
	// Enable AFIO and Port A 
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;
	
	// Configuring PA2 for alternate function output push-pull, max speed 50 MHz
	GPIOA->CRL |= GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2;
	GPIOA->CRL &= ~GPIO_CRL_CNF2_0;
	
	// Configuring PA3 as input pull-up/pull down
	GPIOA->CRL |= GPIO_CRL_CNF3_1;
	GPIOA->CRL &= ~GPIO_CRL_CNF3_0 & ~GPIO_CRL_MODE3;
}

void serial_open(void)
{
	// Enbale USART2 clock
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	// Enbale USART
	USART2->CR1 |= USART_CR1_UE;
	// Set baud rate to 115200bps
	USART2->BRR |= 0x138; //0xEA6;  
	// Enable receiving and transmitting
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;
	// 1 stop bit
	USART2->CR2 &= ~USART_CR2_STOP;
	// set word length to 8 data bits
	USART2->CR1 |= USART_CR1_M;
}

/** Set peripheral back to reset configuration*/ 
void serial_close(void)
{
	RCC->APB1ENR &= ~RCC_APB1ENR_USART2EN;
	USART2->CR1 = 0;
	USART2->BRR =0; 
}

/** Send an 8-bit byte to the serial port
 ** using a bit rate of 115200 bps
 ** Returns 0 if succesful and 1 if not.
 ** Pre-condition: Must have called serial_open()
*/
int send_data(uint8_t data)
{
	if(!(USART2->CR1 & USART_CR1_UE))
	{
		return 1;
	}
	
	while(!(USART2->SR & USART_SR_TXE))
	{
	}
	
	USART2->DR = data;
	return 0;
}

/** Receive an 8-bit byte from the serial port
 ** using a bit rate of 115200 bps
 ** Pre-condition: Must have called serial_open()
*/
uint8_t receive_data(void)
{
	uint8_t value;
	while(!(USART2->SR & USART_SR_RXNE))
	{
	}
	value = USART2->DR;
	return value;
}
void CLI_Transmit(uint8_t *pData, uint16_t Size)
{
		for(uint16_t i = 0; i < Size; i++)
		{
			send_data(pData[i]);
		}
}
void CLI_Receive(uint8_t *pData, uint16_t Size)
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
			send_data(data);  //Echo data
		}
	}
	pData[i] = 0x00;*/
	/*
	for (uint16_t i = 0; i < Size; i++)
	{
		pData[i] = receive_data_uart();
	} */    
}

void print_message(char message[])
{
	CLI_Transmit((uint8_t*) message, strlen(message));
}

