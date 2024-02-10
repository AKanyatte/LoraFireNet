/*
 * http_impl.c
 *
 *  Created on: Feb 10, 2024
 *      Author: kan_a
 */

#include "http_impl.h"

uint8_t buffer[1000] = {0};

/* Transmits and receives data via UART
 */
void Tx_Rx(char *cmd)
{
	memset(buffer, 0, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
	HAL_UART_Receive(&huart1, buffer, 1000, 1000);
	HAL_USART_Transmit(&husart2, buffer, 1000, 1000);
}

/* Checks sim status and if necessary
 *
 */
void check_status(void)
{
	Tx_Rx("AT+CPIN?\r\n");
	Tx_Rx("AT+CSQ\r\n");
	Tx_Rx("AT+CREG?\r\n");
	Tx_Rx("AT+CGDCONT?\r\n");
}

void http_get(void)
{
	char command[300];
	char url[] =  "https://sensorstore-7aa66-default-rtdb.firebaseio.com/.json";
	uint8_t init_is_OK = 0;
	uint8_t term_is_OK = 0;

	while(!init_is_OK)
	{
		Tx_Rx("AT+HTTPINIT\r\n"); //start hhtp service

		if(strstr((char *)buffer,"OK"))
		{
			init_is_OK = 1;
		}

		HAL_Delay(1000);
	}

	sprintf(command, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
	Tx_Rx(command);
	Tx_Rx("AT+HTTPACTION=0\r\n");
	//Tx_Rx("AT+HTTPHEAD\r\n");
	Tx_Rx("AT+HTTPREAD?\r\n");

	while(!term_is_OK)
	{
		Tx_Rx("AT+HTTPTERM\r\n");  //stop http service
		if(strstr((char *)buffer,"OK"))
		{
			term_is_OK = 1;
		}
		HAL_Delay(1000);

	}
}

void http_post(void)
{
	char command[300];
	char url[] =  "https://sensorstore-7aa66-default-rtdb.firebaseio.com/.json";
	char content[] = "application/json";
	uint8_t init_is_OK = 0;
	uint8_t term_is_OK = 0;
	int humidity_val = 85;
	int temp_val = 25;
	int smoke_val = 450;

	while(!init_is_OK)
	{
		Tx_Rx("AT+HTTPINIT\r\n"); //start hhtp service

		if(strstr((char *)buffer,"OK"))
		{
			init_is_OK = 1;
		}

		HAL_Delay(1000);
	}

	sprintf(command, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
	Tx_Rx(command);
	sprintf(command,"AT+HTTPPARA=\"CONTENT\",\"%s\"\r\n", content);
	Tx_Rx(command);
	Tx_Rx("AT+HTTPDATA=50,10000\r\n");
	sprintf(command,"{\"humidity\": %d, \"smoke\": %d, \"temperature\": %d}\r\n", humidity_val, smoke_val, temp_val);
	Tx_Rx(command);
	HAL_Delay(5000);
	Tx_Rx("AT+HTTPACTION=1\r\n");
	HAL_Delay(2000);
	Tx_Rx("AT+HTTPREAD?\r\n");
	HAL_Delay(2000);

	while(!term_is_OK)
	{
		Tx_Rx("AT+HTTPTERM\r\n");  //stop http service
		if(strstr((char *)buffer,"OK"))
		{
			term_is_OK = 1;
		}
		HAL_Delay(1000);
	}
}




