/*
 * pk.h
 *
 *  Created on: Mar 12, 2024
 *      Author: kan_a
 */

#include <stdint.h>
#include "stm32l4xx.h"
#include "packet_formats.h"

#ifndef INC_PK_H_
#define INC_PK_H_

//NODE SPECIFIC DATA
	uint32_t my_id; 	//this node's ID
	uint32_t my_sequence_number; 	//this node's sequence number
	uint32_t rreq_id; 				//id for RREQ

	//Setup Data
	#define UNICAST_TABLE_LENGTH 16
	#define NOROUTE_QUEUE_MAX_ENTRIES 5
	#define ROUTE_LIFETIME 100;
	#define DEFAULT_ROUTE_EXPIRATION_TIME 1000

	//Packet types
	#define DATA_PACKET 0
	#define RREQ_PACKET 1
	#define RREP_PACKET 2
	#define RERR_PACKET 3
	#define INVALID_PACKET 0xF0;

	//Route destinations
	#define NO_ROUTE 0

	//locations for holding data
	uint8_t rx_data[256];			//Buffer for holding received data

	//route table unicast
	struct unicast_route_table_entry
	{
		uint32_t destination_id;
		uint32_t destination_sequence_number;
		uint32_t hop_count;
		uint32_t next_hop_destination_id;
		uint32_t * precursor_nodes_destination_id_array;
		uint32_t expiration_time;
	};
	uint32_t unicast_entries;

	struct unicast_route_table_entry unicast_route_table[UNICAST_TABLE_LENGTH];

	//no route table
	struct noroute_table_entry
	{
		uint32_t destination_id;
		uint8_t * data;
		uint8_t data_length;
	};

	struct noroute_table_entry noroute_table[NOROUTE_QUEUE_MAX_ENTRIES];						//TODO, this needs to be a linkedlist
	uint8_t noroute_table_entries;

	#define RREQ_TABLE_MAX_ENTRIES 10
	uint32_t rreq_table[RREQ_TABLE_MAX_ENTRIES];
	uint8_t rreq_pointer;



#endif /* INC_PK_H_ */
