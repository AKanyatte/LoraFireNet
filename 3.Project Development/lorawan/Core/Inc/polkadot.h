/*
 * header file for polkadot driver. Includes major data fields for the AODV implementation
 */
#include <stdint.h>
#include "pk.h"
#include "stm32l4xx.h"
#include "packet_formats.h"

#ifndef SRC_POLKADOT_H_
#define SRC_POLKADOT_H_



	//FUNCTION PROTOTYPES
	uint8_t mesh_transmit(uint32_t destination_id, uint8_t data[], uint8_t data_length);
	uint8_t mesh_send_data(uint32_t destination_id, uint32_t dest_seq_num, uint8_t * data, uint32_t receiver, uint8_t num_hops, uint32_t source_id, uint8_t data_length);
	uint8_t mesh_send_rreq(uint32_t destination_id, uint32_t source_id, uint32_t source_sequence_number, uint8_t num_hops, uint32_t rreq_id);
	uint8_t mesh_send_rrep(uint32_t receiver_id, uint32_t destination_id, uint32_t source_id, uint8_t num_hops, uint32_t dest_seq_num);
	uint8_t mesh_unicast(uint32_t destination_id, uint32_t dest_seq_num, uint8_t * data, uint32_t receiver, uint8_t num_hops, uint32_t source_id, uint8_t data_length);
	uint8_t format_packet_data(struct data_packet packet, uint8_t packet_arr[]);
	uint8_t format_packet_rreq(struct rreq_packet packet, uint8_t packet_arr[]);
	uint8_t format_packet_rrep(struct rrep_packet packet, uint8_t packet_arr[]);
	struct unicast_packet unpack_packet_unicast(uint8_t parr[], uint8_t data_length, uint8_t data[]);
	uint8_t receive_packet_handler(uint8_t packet_data[], uint8_t plength);
	uint8_t packet_type(uint8_t packet_data[]);
	int8_t route_exists(uint32_t id);
	uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt);
	uint8_t polkadot_init(uint32_t id);
	uint8_t rreq_table_contains(uint32_t rreq_id);
	uint8_t rreq_table_append(uint32_t rreq_id);
	uint8_t update_route_table(uint32_t dest_id, uint32_t dest_seq_num, uint8_t num_hops, uint32_t next_hop);
	uint8_t mesh_send_hello();
	void rand_delay();
	uint32_t get_UID();


#endif /* SRC_POLKADOT_H_ */
