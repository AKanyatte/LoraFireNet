/*
 * internet_impl.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 *	Implementation of functions for managing TCP/IP services
 *
 *
 */

#include "internet_impl.h"

/**
 * Starts TCP/IP service
 * This function sends AT commands to initiate the
 * TCP/IP service and waits for confirmation of connection
 *
 * Returns:
 * 	Null
 */
void start_service(void)
{
	uint8_t NETOPEN_is_OK = 0;

	while(!NETOPEN_is_OK)
	{
		Tx_Rx("AT+NETOPEN\r\n");
		HAL_Delay(1000);

		HAL_Delay(1000);
		Tx_Rx("AT+NETOPEN?\r\n");
		HAL_Delay(1000);

		if(strstr((char *)buffer,"+NETOPEN: 1"))
		{
			NETOPEN_is_OK = 1;
		}

		HAL_Delay(1000);

	}
}

/**
 * Ends TCP/IP service
 * This function sends AT commands to stops the
 * TCP/IP service and waits for confirmation.
 *
 * Returns:
 * 	Null
 */
void stop_service(void)
{
	uint8_t NETCLOSE_is_OK = 0;

	while(!NETCLOSE_is_OK)
	{
		Tx_Rx("AT+NETCLOSE\r\n");
		HAL_Delay(1000);

		if(strstr((char *)buffer,"OK"))
		{
			NETCLOSE_is_OK = 1;
		}

		HAL_Delay(1000);
	}
}

/**
 * Obtain IP address for the 4G/LTE module
 *
 * Returns:
 * 	Null
 */
void get_ipaddr(void)
{
	Tx_Rx("AT+IPADDR\r\n");
}

