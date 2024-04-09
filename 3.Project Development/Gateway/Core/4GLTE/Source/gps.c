/*
 * gps.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 * Functions for managing GPS communication with the SIM7600X module
 *
 *
 */

#include "gps.h"

/**
 * Starts GPS session
 *
 * Returns:
 * 	Null
 */
void start_gps(void)
{
	uint8_t gps_is_OK = 0;
	Tx_Rx("AT+CGPS=1");

	while(!gps_is_OK)
	{
		Tx_Rx("AT+CGPS=1\r\n");	//start gps session

		if(strstr((char *)buffer,"OK"))
		{
			gps_is_OK = 1;
		}
		HAL_Delay(1000);
	}
}

/**
 * Ends the current GPS session
 *
 * Returns:
 * 	Null
 */
void end_gps(void)
{
	uint8_t gps_is_OK = 0;
	Tx_Rx("AT+CGPS=0");

	while(!gps_is_OK)
	{
		Tx_Rx("AT+CGPS=0\r\n");	//start gps session

		if(strstr((char *)buffer,"OK"))
		{
			gps_is_OK = 1;
		}
		HAL_Delay(1000);
	}
}

/**
 * Retrieves GPS information
 *
 * Returns:
 * 	GPS information string, containing GPS coordinates.
 */
char* get_gps_info(void)
{
	static char gps_info[256];
	gps_info[0] = '\0';

	Tx_Rx("AT+CGPSINFO\r\n");

	// Find the position of the colon in the response
	char *colon_position = strchr((char*)buffer, ':');

	if (colon_position != NULL)
	{
	    // Initialize a counter to track the number of commas encountered
	    int comma_count = 0;
	    // Iterate through the string starting from the position of the colon
	    for (char *current_position = colon_position; *current_position != '\0'; current_position++)
	    {
	    	// If a comma is encountered, increment the comma count
	        if (*current_position == ',')
	        {
	            comma_count++;
	            // If the fourth comma is reached, extract the substring
	            if (comma_count == 4)
	            {
	                // Copy the substring between the colon and the fourth comma
	                strncpy(gps_info, colon_position + 1, current_position - colon_position - 1);
	                gps_info[current_position - colon_position - 1] = '\0'; // Null-terminate the string
	                break;
	            }
	        }
	    }
	}
	return gps_info;
}
