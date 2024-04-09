/*
 * send_data.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 * Contains functions necessary for sending AT commands via
 * UART to the SIM7600X module.
 * Also includes the implementation of configuring SMS capability on
 * the SIM7600X module, in order to send SMS alerts
 *
 */


#include "send_data.h"
#include "gps.h"
#include "http_impl.h"
#include "internet_impl.h"


uint8_t buffer[1000] = {0};

/**
 * General function for transmitting and receicing AT commands
 * over UART.
 *
 * Parameters:
 * 	cmd: Command to transmit
 *
 * Returns:
 *	Null
 */
void Tx_Rx(char *cmd)
{
	memset(buffer, 0, sizeof(buffer));
	HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 1000);
	HAL_UART_Receive(&huart1, buffer, 1000, 1000);
	HAL_USART_Transmit(&husart2, buffer, 1000, 1000);
}

/**
 * Send AT command to check if module is ready
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 *	Null
 */
void send_at(void)
{
	uint8_t AT_is_OK = 0;

	while(!AT_is_OK)
	{
		Tx_Rx("AT\r\n");
		HAL_Delay(1000);

		if(strstr((char *)buffer,"OK"))
		{
			AT_is_OK = 1;
		}
		HAL_Delay(1000);
	}
}

/**
 * Sets sms mode for sending text messages
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 *	Null
 */
void set_sms_mode(void)
{
	uint8_t text_mode_set = 0;

	while(!text_mode_set)
	{
		Tx_Rx("AT+CMGF=1\r\n");

		if(strstr((char *)buffer,"OK"))
		{
			text_mode_set = 1;
		}

		HAL_Delay(1000);
	}
}


/**
 * Send text alert
 *
 * Parameters:
 * 	alert: Message to send
 *
 * Returns:
 *	Null
 */

void send_sms(char *alert)
{
	char AT_command[500];
	uint8_t rx_buffer[100] = {0};
	char mobile_num[20] = "4039034943";

	sprintf(AT_command,"AT+CMGS=\"%s\"\r\n", mobile_num);
	Tx_Rx(AT_command);
	HAL_Delay(100);

	//memset(buffer,0,sizeof(buffer));
	HAL_UART_Transmit(&huart1,(uint8_t *)alert,strlen(alert),1000);
	HAL_USART_Transmit(&husart2,(uint8_t *)alert,strlen(alert),1000);
	HAL_UART_Receive (&huart1,rx_buffer, 100, 100);
	HAL_USART_Transmit(&husart2,rx_buffer,100,100);
	HAL_Delay(10000);
}

/**
 * Sends alert message based on wildfire prediction
 *
 * Parameters:
 * 	Null
 *
 * Returns:
 *	Null
 */
void send_alert(void)
{
	float prediction;
	char message[500];
	char *gps;

	prediction = get_prediction();
	HAL_Delay(2000);
	stop_service();
	HAL_Delay(2000);
	set_sms_mode();
	start_gps();
	HAL_Delay(1000);
	gps = get_gps_info();

	if(prediction >= HIGHLY_PROBABLE)
	{
		snprintf(message, sizeof(message), "Urgent attention required! A wildfire is highly probable. Location:%s%c", gps, 0x1A);
	}
	else if(prediction >= IMMINENT_WILDFIRE)
	{
		snprintf(message, sizeof(message), "A wildfire is imminent. Location:%s%c", gps, 0x1A);
	}
	else if(prediction >= HIGH_LIKELIHOOD)
	{
		snprintf(message, sizeof(message), "There is a high likelihood of fire. Location:%s%c", gps, 0x1A);
	}
	else if(prediction >= MILD_CHANCE)
	{
		snprintf(message, sizeof(message), "There is a mild chance of fire. Location:%s%c", gps, 0x1A);
	}
	else
	{
		end_gps();
	}

	send_sms(message);
	end_gps();
}







