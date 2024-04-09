/*
 * packets.c
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 *
 * This file contains functions for handling packets in a LoRa mesh network.
 * It includes functions for checking if a packet is intended for the current module,
 * determining the next hop for the packet based on mesh routing configuration,
 * transmitting packets to the next hop, handling received packets, processing received packets,
 * and sending packets.
 *
 */


#include "packets.h"
#include "http_impl.h"
#include "internet_impl.h"
#include "send_data.h"

extern USART_HandleTypeDef husart2;
uint8_t processed_packet[4];
uint8_t processed_packet_length = 0;


/**
 * Checks if the packet is intended for this module.
 *
 * Parameters:
 * 	dest_address:	The destination address extracted from the packet.
 *	my_address:	The address of this module.
 *
 * Returns:
 * 	true if the packet is intended for this module, false otherwise.
 */

bool is_packet_for_this_module(uint8_t dest_address, uint8_t my_address)
{
    // Extract destination address from the packet
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


/**
 * Determines the next hop based on the destination address and mesh configuration.
 *
 * Parameters:
 *  payload: The packet payload.
 *  my_address: The address of this module.
 *
 * 	Returns:
 * 		the next hop address.
 */

uint8_t determine_next_hop(uint8_t *payload, uint8_t my_address)
{
    // Mesh routing configuration
    if (my_address == GATEWAY_ADDRESS) {
        // If this node is the master node, route all packets to itself
        return GATEWAY_ADDRESS; }
    else if (my_address == NODE_2_ADDRESS) {
        // If this node is node 2, route packets to node 3
        return NODE_3_ADDRESS; }
    else if((my_address == NODE_1_ADDRESS) || (my_address == NODE_3_ADDRESS)){
        // For nodes 1 and 3, route packets to the master node
        return GATEWAY_ADDRESS; }
    else {
    	 // No next hop, packet should be discarded
    	return NO_NEXT_HOP;
    }
}


/**
 * Transmits packet to the next hop.
 *
 * Parameters:
 * 	lora: LoRa struct
 * 	payload:  The packet payload.
 * 	len: The length of the payload.
 * 	next_hop: The next hop address.
 *
 * Returns:
 * 	Null
 *
 */

void transmit_packet_to_next_hop(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t next_hop)
{
    lora_send_packet(lora, payload, len);
}


/**
 * Handles the received packet
 *
 * Parameters:
 * 	lora: LoRa struct
 * 	payload:  The packet payload.
 * 	len: The length of the payload.
 * 	address: The address of this module.
 *
 * Returns:
 * 	Null
 *
 */

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
        uint8_t next_hop = determine_next_hop(payload, address);

        snprintf(message, sizeof(message),"Packet is being transmitted to next hop: %x\r\n", next_hop);
        HAL_USART_Transmit(&husart2,(uint8_t *)message,strlen(message), 100);
        HAL_Delay(1000);

        if (next_hop != NO_NEXT_HOP)
        {
            transmit_packet_to_next_hop(lora, payload, len, next_hop);
        }
    }
}

/**
 * Process the received packet
 *
 * Parameters:
 * 	payload:  The packet payload.
 * 	len: The length of the payload.
 *
 * Returns:
 * 	Null
 */

void process_packet(uint8_t *payload, uint8_t len)
{
	char packet[200];
	uint8_t rh = payload[1];
	uint8_t smoke = payload[2];
	uint8_t temp = payload[3];

	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1 ) == 0x00)
	{
		HAL_GPIO_WritePin(GPIOB, LD3_Pin, GPIO_PIN_RESET);
	}
	for (int i = 0; i < len; ++i)
	{
		snprintf(packet, sizeof(packet),"%d \r\n", payload[i]);
	    HAL_USART_Transmit(&husart2,(uint8_t *)packet,strlen(packet), 100);
	    HAL_Delay(1000);
	}

	// Using 4G/LTE to post data and send alerts
	check_status();
	start_service();
	get_ipaddr();
	http_get();
	http_post(rh, smoke, temp);
	HAL_Delay(10000);
	send_alert();
}


/**
 * Sends a packet
 *
 * Parameters:
 * 	lora: LoRa struct
 * 	data: The packet data
 * 	len: The length of the packet data
 *
 * Returns:
 * 	Null
 */
void send_packet(lora_sx1276 *lora, uint8_t *data, uint8_t data_len)
{
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
}

