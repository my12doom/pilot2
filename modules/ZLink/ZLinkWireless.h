#pragma once

#include <stdint.h>

#pragma pack(push, 1)

typedef struct
{
	uint16_t crc;			// crc16 of remaining data
	uint8_t type;
	uint8_t length;
	uint16_t src_node_id;
	uint16_t dest_node_id;
	uint16_t sequence_id;
	uint8_t payload[];
} WirelessPacket;

#pragma pack(pop)

#define PACKET_HEADER_SIZE sizeof(WirelessPacket)
#define NODE_BROADCAST 0xffff
#define NODE_STATION 0

enum WirelessPacketType
{
	// station packets
	packet_probe = 0,

	// node packets
	packet_probe_respond = 1,

	// common
	packet_management = 2,
	packet_payload = 3,
	packet_ack = 4,
	packet_payload_rtk = 5,
	packet_timing = 6,
	packet_result = 7,

	packet_flag_need_ack = 0x80,
};

void update_crc(WirelessPacket * frame);
bool check_crc(WirelessPacket *frame);
WirelessPacket *new_packet(uint16_t data_size, uint16_t sequence_id, uint8_t type, uint16_t src_node_id, uint16_t des_nodeid, const uint8_t *data);
WirelessPacket *clone_packet(WirelessPacket *frame);
void free_packet(WirelessPacket *p);
