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
#define GATEWAY_ADDRESS		0x01
#define NODE_1_ADDRESS		0x02
#define NODE_2_ADDRESS		0x03
#define NODE_3_ADDRESS		0x04
#define NO_NEXT_HOP			0x00


/** Checks if the packet is intended for this module
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


/** Determine the next hop based on the destination address and mesh configuration
 */
uint8_t determine_next_hop(uint8_t *payload, uint8_t my_address)
{
    // Extract destination address from the packet
    uint8_t dest_address = payload[0];

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


/** Transmit packet to the next hop
 */
void transmit_packet_to_next_hop(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t next_hop)
{
    lora_send_packet(lora, payload, len);
}


/** Handles the received packet
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

/** Process the received packet
 */
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

/** Sends a packet
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

