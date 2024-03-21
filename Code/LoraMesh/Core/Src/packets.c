/*
 * packets.c
 *
 *  Created on: Mar 14, 2024
 *      Author: kan_a
 */


#include "packets.h"

extern USART_HandleTypeDef husart2;

#define MAX_ADDRESS 4 // Maximum number of modules
#define MAX_PACKET_SIZE 64 // Maximum packet size


// Define addresses for each module
#define MODULE_1_ADDRESS 0x01
#define MODULE_2_ADDRESS 0x02
#define MODULE_3_ADDRESS 0x03
#define MODULE_4_ADDRESS 0x04
#define NO_NEXT_HOP		 0x00


// Function to check if the packet is intended for this module
bool is_packet_for_this_module(uint8_t dest_address, uint8_t my_address)
{
    // Extract destination address from the packet
   // uint8_t dest_address = payload[0]; // First byte is the destination address
    // Compare with the address of this module
    if(dest_address == my_address)
    {
    	return true;
    }
    else
    {
    	return false;
    }
}

// Function to determine the next hop based on the destination address
uint8_t determine_next_hop(uint8_t *payload)
{
    // Extract destination address from the packet
    uint8_t dest_address = payload[0]; // Assuming the first byte is the destination address

    // Simple routing: Forward packet to next module
    if (dest_address == MODULE_1_ADDRESS)
    {
        return MODULE_2_ADDRESS;
    }
    else if (dest_address == MODULE_2_ADDRESS)
    {
        return MODULE_3_ADDRESS;
    }
    else if (dest_address == MODULE_3_ADDRESS)
    {
    	return MODULE_2_ADDRESS;
    }
    else
    {
        // No next hop, packet should be discarded
    	return NO_NEXT_HOP;
    }
}

// Function to transmit the packet to the next hop module
void transmit_packet_to_next_hop(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t next_hop)
{
    lora_send_packet(lora, payload, len);
}


void handle_received_packet(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t address)
{
	char message[200];
    // Check if the packet is intended for this module
    if (is_packet_for_this_module(payload[0], address))
    {
        // Process the packet
        process_packet(payload, len);
    }
    else
    {
        // Forward the packet to the next hop
        uint8_t next_hop = determine_next_hop(payload);

        snprintf(message, sizeof(message),"Packet is being transmitted to next hop: %x\r\n", next_hop);
        HAL_USART_Transmit(&husart2,(uint8_t *)message,strlen(message), 100);
        HAL_Delay(1000);

        if (next_hop != NO_NEXT_HOP)
        {
            transmit_packet_to_next_hop(lora, payload, len, next_hop);
        }
    }
}

// Function to process the received packet
void process_packet(uint8_t *payload, uint8_t len)
{
	char packet[200];
	//Print the received packet payload
	snprintf(packet, sizeof(packet), "Received packet: ");
	for (int i = 0; i < len; ++i)
	    {
	        snprintf(packet, sizeof(packet),"%d \r\n", payload[i]);
	        HAL_USART_Transmit(&husart2,(uint8_t *)packet,strlen(packet), 100);
	        HAL_Delay(1000);
	    }
    // TODO: Process the received packet into array of sorts?
}

// Function to get the address of this module
uint8_t get_module_address()
{
    //Each module has a unique address (1 to 4)
    return 0x01; // Change this value for each module
}


// Function to send a packet
void send_packet(lora_sx1276 *lora, uint8_t *data, uint8_t data_len)
{
    //uint8_t buffer[MAX_PACKET_SIZE];
    //buffer[0] = destination_address; // Set the destination address
    //memcpy(buffer + 1, data, data_len); // Copy the payload
    // Send packet
	char message[200];
	uint8_t res = lora_send_packet(lora, data, data_len);
	if (res != LORA_OK)
	{
		// Send failed
		snprintf(message, sizeof(message),"Send failed \r\n");
		HAL_USART_Transmit(&husart2,(uint8_t *)message,strlen(message), 100);
		HAL_Delay(1000);
	}
	else
	{
		snprintf(message, sizeof(message),"Send succeeded \r\n");
		HAL_USART_Transmit(&husart2,(uint8_t *)message,strlen(message), 100);
		HAL_Delay(1000);
	}
     // Include the destination address
}

