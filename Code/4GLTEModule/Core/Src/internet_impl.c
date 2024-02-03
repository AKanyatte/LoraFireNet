/*
 * internet_impl.c
 *
 *  Created on: Feb 3, 2024
 *      Author: kan_a
 */

#include "internet_impl.h"

void start_service(void)
{
	char AT_command[20];
	uint8_t rx_buffer[20] = {0};

	sprintf(AT_command,"AT+HTTPINIT\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(1000);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		start_service();
	}
	HAL_Delay(1000);
}

void stop_service(void)
{
	char AT_command[20];
	uint8_t rx_buffer[20] = {0};

	sprintf(AT_command,"AT+HTTPTERM\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(1000);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		stop_service();
	}

	HAL_Delay(1000);
}

void access_server(void)
{

}



