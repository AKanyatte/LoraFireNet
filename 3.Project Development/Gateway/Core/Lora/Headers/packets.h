/*
 * packets.h
 *
 * Author: Ashley Kanyatte
 *
 * Description:
 * This file contains function prototypes for packets.c as well as defined constants used by these functions.
 *
 */

#ifndef INC_PACKETS_H_
#define INC_PACKETS_H_

#include "lora_sx1276.h"
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define MAX_ADDRESS 4 // Maximum number of modules
#define MAX_PACKET_SIZE 64 // Maximum packet size

// Define addresses for each module
#define GATEWAY_ADDRESS		0x01
#define NODE_1_ADDRESS		0x02
#define NODE_2_ADDRESS		0x03
#define NODE_3_ADDRESS		0x04
#define NO_NEXT_HOP			0x00


// Function to check if the packet is intended for this module
bool is_packet_for_this_module(uint8_t dest_addr, uint8_t my_address);

// Function to determine the next hop based on the destination address
uint8_t determine_next_hop(uint8_t *payload, uint8_t my_address);

// Function to transmit the packet to the next hop module
void transmit_packet_to_next_hop(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t next_hop);

// Function to handle packet received by a node
void handle_received_packet(lora_sx1276 *lora, uint8_t *payload, uint8_t len, uint8_t address);

// Function to process the received packet
void process_packet(uint8_t *payload, uint8_t len);

// Function to get the address of this module
uint8_t get_module_address();

// Function to send a packet
void send_packet(lora_sx1276 *lora, uint8_t *data, uint8_t data_len);


#endif /* INC_PACKETS_H_ */
