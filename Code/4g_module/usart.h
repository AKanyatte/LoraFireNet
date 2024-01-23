#include "stm32f10x.h"
#include <string.h>
#include <stdio.h>

void clockInit(void);
void usart_config(void);
void send_data_uart(uint8_t x);
uint8_t receive_data_uart(void);
void msg_transmit(uint8_t *pData, unsigned int Size);
void msg_receive(uint8_t *pData, unsigned int Size);
void delay(uint32_t time_delay);
void led_init(void);
void led_on(void);
void led_off(void);
