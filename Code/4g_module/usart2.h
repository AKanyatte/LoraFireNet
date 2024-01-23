#include "stm32f10x.h"

void usart2_config(void);
void serial_open(void);
void serial_close(void);
int send_data(uint8_t data);
uint8_t receive_data(void);
void CLI_Transmit(uint8_t *pData, uint16_t Size);
void CLI_Receive(uint8_t *pData, uint16_t Size);
void print_message(char message[]);
