/*
 * send_data.c
 *
 *  Created on: Feb 3, 2024
 *      Author: kan_a
 */
#include "send_data.h"


void send_at(void)
{
	char AT_command[20];
	uint8_t AT_is_OK = 0;
	uint8_t rx_buffer[30] = {0};

	while(!AT_is_OK)
	{
		sprintf(AT_command, "AT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
		HAL_Delay(1000);

		if(strstr((char *)rx_buffer,"\r\nOK\r\n"))
		{
			AT_is_OK = 1;
		}
		else
		{
			memset(rx_buffer,0,sizeof(rx_buffer));
			send_at();
		}
		HAL_Delay(1000);
	}
}

void set_sms_mode(void)
{
	char AT_command[20];
	uint8_t rx_buffer[30] = {0};

	sprintf(AT_command,"AT+CMGF=1\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(1000);

	if(strstr((char *)rx_buffer,"\r\nOK\r\n"))
	{}
	else
	{
	    memset(rx_buffer,0,sizeof(rx_buffer));
		set_sms_mode();
	}
}

void send_sms(void)
{
	char AT_command[300];
	char mobile_num[20] = "4039034943";
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,"AT+CMGS=\"%s\"\r\n", mobile_num);
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(100);
	memset(rx_buffer,0,sizeof(rx_buffer));

	sprintf(AT_command,"Hello from STM%c", 0x1A);
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"+CMGS"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		send_sms();
	}
	HAL_Delay(4000);
}







