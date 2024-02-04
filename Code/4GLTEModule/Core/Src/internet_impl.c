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

	sprintf(AT_command,"AT+NETOPEN\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	sprintf(AT_command,"AT+NETOPEN?\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);
	HAL_Delay(1000);

	if(strstr((char *)rx_buffer,"+NETOPEN: 1"))
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
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command,"AT+NETCLOSE\r\n");
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

void set_pdp(void)
{
	char AT_command[100];
	char apn[] = "pda.stm.sk.ca";
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,"AT+CGDCONT=1,\"IP\",\"%s\",\"0.0.0.0\",0,0\r\n",apn);
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
	    memset(rx_buffer,0,sizeof(rx_buffer));
		set_pdp();
	}
}

void select_application_mode(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command,"AT+CIPMODE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);

	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}

}

void set_send_mode(void)
{
	char AT_command[50];
	uint8_t rx_buffer[50] = {0};

	sprintf(AT_command,"AT+CIPSENDMODE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}

}

void configure_param(void)
{
	char AT_command[100];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,	"AT+CIPCCFG=10,0,0,0,1,0,30000\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}

}

void set_timeout(void)
{
	char AT_command[100];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command,	"AT+CIPTIMEOUT=30000,15000,15000\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}

}

void get_ipaddr(void)
{
	char AT_command[50];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command, "AT+IPADDR\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"+IPADDR"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}

}

void setup_tcp_connection(void)
{
	char AT_command[200];
	char server[] = "https://drive.google.com/drive/folders/1-ngEewE-qfFc6WE9AQIlLAqVE7q_hGha";
	const int port = 80;
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command, "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",server,port);
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}
}

void destroy_tcp_connection(void)
{
	char AT_command[50];
	uint8_t rx_buffer[100] = {0};

	sprintf(AT_command, "AT+CIPCLOSE=0\r\n");
	HAL_UART_Transmit(&huart1,(uint8_t *)AT_command,strlen(AT_command),1000);
	HAL_UART_Receive (&huart1, rx_buffer, sizeof(rx_buffer), 100);

	if(strstr((char *)rx_buffer,"OK"))
	{}
	else
	{
		memset(rx_buffer,0,sizeof(rx_buffer));
		select_application_mode();
	}
}

