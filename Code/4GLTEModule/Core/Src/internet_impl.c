/*
 * internet_impl.c
 *
 *  Created on: Feb 3, 2024
 *      Author: kan_a
 */

#include "internet_impl.h"

void start_service(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};
	uint8_t NETOPEN_is_OK = 0;

	while(!NETOPEN_is_OK)
	{
		sprintf(AT_command,"AT+NETOPEN\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_UART_Receive (&huart1, rx_buffer, 50, 100);
		HAL_USART_Transmit(&husart2,rx_buffer,50,100);
		HAL_Delay(1000);
		memset(rx_buffer,0,sizeof(rx_buffer));

		sprintf(AT_command,"AT+NETOPEN?\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_UART_Receive (&huart1, rx_buffer, 50, 100);
		HAL_USART_Transmit(&husart2,rx_buffer,50,100);
		HAL_Delay(1000);

		if(strstr((char *)rx_buffer,"+NETOPEN: 1"))
		{
			NETOPEN_is_OK = 1;
		}

		HAL_Delay(1000);

	}
}

void stop_service(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};
	uint8_t NETCLOSE_is_OK = 0;

	while(!NETCLOSE_is_OK)
	{
		sprintf(AT_command,"AT+NETCLOSE\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_UART_Receive (&huart1, rx_buffer, 50, 100);
		HAL_USART_Transmit(&husart2,rx_buffer,50,100);
		HAL_Delay(1000);

		if(strstr((char *)rx_buffer,"OK"))
		{
			NETCLOSE_is_OK = 1;
		}

		HAL_Delay(1000);
	}
}



void set_pdp(void)
{
	char AT_command[100];
	char apn[] = "inet.stm.sk.ca";//"pda.stm.sk.ca";
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,"AT+CGDCONT=1,\"IPV4V6\",\"%s\",\"0.0.0.0\",0,0\r\n",apn);
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,100,100);

}

void select_application_mode(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command,"AT+CIPMODE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,50,100);

}

void set_send_mode(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command,"AT+CIPSENDMODE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 50, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,50,100);
}

void configure_param(void)
{
	char AT_command[100];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,	"AT+CIPCCFG=10,0,0,0,1,0,75000\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,100,100);
}

void set_timeout(void)
{
	char AT_command[100];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,	"AT+CIPTIMEOUT=75000,15000,15000\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,100,100);

}

void get_ipaddr(void)
{
	char AT_command[50];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command, "AT+IPADDR\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,100,100);

}

void setup_tcp_connection(void)
{
	char AT_command[200];
	char server[] = "firebase.googleapis.com";
	const int port = 80;
	uint8_t rx_buffer[100] = {0};
	//uint8_t CIPOPEN_is_OK = 0;

	//while(!CIPOPEN_is_OK)
	//{
		sprintf(AT_command, "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",server,port);
		HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
		HAL_UART_Receive (&huart1, rx_buffer, 100, 100);
		HAL_USART_Transmit(&husart2,rx_buffer,100,100);
		HAL_Delay(1000);

		//if(strstr((char*)rx_buffer, "+CIPOPEN: 0,0"))
		//{
		//	CIPOPEN_is_OK = 1;
		//}

	//}
}

void destroy_tcp_connection(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command, "AT+CIPCLOSE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, 50, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,50,100);

}

