/*
 * http_impl.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 *	Implementation of functions for managing HTTP requests communication
 */

#include "http_impl.h"


/**
 * Checks SIM status
 *
 * Returns:
 * 	Null
 */
void check_status(void)
{
	Tx_Rx("AT+CPIN?\r\n");
	Tx_Rx("AT+CSQ\r\n");
	Tx_Rx("AT+CREG?\r\n");
	Tx_Rx("AT+CGDCONT?\r\n");
}

/**
 * Performs an HTTP GET request to a remote Firebase Realtime Database,
 * used to store sensor data
 *
 * Returns:
 * 	Null
 */
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

/**
 * Performs an HTTP POST request to a remote Firebase Realtime Database,
 * used to store sensor data, posts the received sensor data.
 *
 * Parameters:
 * humidity_val: Humidity value
 * smoke_val: Smoke value
 * temp_val: Temperature value
 *
 * Returns:
 * 	Null
 */
void http_post(uint8_t humidity_val, uint8_t smoke_val, uint8_t temp_val)
{
	char command[300];
	char url[] = "https://sensorstore-7aa66-default-rtdb.firebaseio.com/.json";
	char content[] = "application/json";
	uint8_t init_is_OK = 0;
	uint8_t term_is_OK = 0;

	while(!init_is_OK)
	{
		Tx_Rx("AT+HTTPINIT\r\n"); //start hhtp servicen

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
	HAL_Delay(2000);
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


/**
 * Performs an HTTP POST request to a remote Firebase Realtime Database,
 * used to store predictions. Retrieves the last predicted value.
 *
 * Returns:
 * 	Last predicted value
 */
float get_prediction(void)
{
    char command[300];
    char url[] = "https://predictions-ba982-default-rtdb.firebaseio.com/.json";

    uint8_t init_is_OK = 0;
    uint8_t term_is_OK = 0;
    const char *start_ptr;
    const char *end_ptr;

    while (!init_is_OK)
    {
        Tx_Rx("AT+HTTPINIT\r\n"); // Start HTTP service

        if (strstr((char *)buffer, "OK"))
        {
            init_is_OK = 1;
        }

        HAL_Delay(1000);
    }

    sprintf(command, "AT+HTTPPARA=\"URL\",\"%s\"\r\n", url);
    Tx_Rx(command);
    Tx_Rx("AT+HTTPACTION=0\r\n");
    Tx_Rx("AT+HTTPREAD=0,1000\r\n");
    HAL_Delay(2000);

    // Find the last occurrence of "{" to start parsing the last entry
    const char *last_entry_start = strrchr((char*)buffer, '{');
    if (last_entry_start != NULL)
    {
    	// Find the end of the last entry
    	const char *last_entry_end = strchr(last_entry_start, '}');
    	if (last_entry_end != NULL)
    	{
    		// Allocate memory for the last entry
    	    size_t entry_length = last_entry_end - last_entry_start + 1;
    	    char *last_entry = (char *)malloc(entry_length + 1);
    	    if (last_entry != NULL)
    	    {
    	    	// Copy the last entry into a new buffer
    	    	strncpy(last_entry, last_entry_start, entry_length);
    	    	last_entry[entry_length] = '\0';
    	    	// Find the "predicted_probability" field in the last entry
    	    	const char *prediction_start = strstr(last_entry, "\"predicted_probability\":");
    	    	if (prediction_start != NULL)
    	    	{
    	    		 // Extract the predicted probability value
    	    		 prediction_start += strlen("\"predicted_probability\":");
    	    		 float prediction = atof(prediction_start);

    	    		 // Free the allocated memory
    	    		 free(last_entry);

    	    		 // Return the prediction value
    	    		 return prediction;
    	    	}
    	    	// Free the allocated memory
    	    	free(last_entry);
    	    }
    	}
    }
    while (!term_is_OK)
    {
        Tx_Rx("AT+HTTPTERM\r\n"); // Stop HTTP service
        if (strstr((char *)buffer, "OK"))
        {
            term_is_OK = 1;
        }
        HAL_Delay(1000);
    }

    return -1.0f; // Return default value if prediction extraction fails
}


