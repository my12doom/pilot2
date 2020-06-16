#include "ZLinkWireless.h"
#include <modules/utils/crc16.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

void update_crc(WirelessPacket * p)
{
	uint16_t crc = crc16((unsigned char*)&(p->type), PACKET_HEADER_SIZE - 2 + p->length);
	p->crc = crc;
}
bool check_crc(WirelessPacket *p)
{
	uint16_t crc = crc16((unsigned char*)&(p->type), PACKET_HEADER_SIZE - 2 + p->length);
	return crc == p->crc;
}
WirelessPacket *new_packet(uint16_t data_size, uint16_t sequence_id, uint8_t type, uint16_t src_nodeid, uint16_t des_nodeid, const uint8_t *data)
{
	WirelessPacket * p = (WirelessPacket*)malloc(data_size + PACKET_HEADER_SIZE);
	if (!p)
		return NULL;

	p->type = type;
	p->length = data_size;
	p->dest_node_id = des_nodeid;
	p->src_node_id = src_nodeid;
	p->sequence_id = sequence_id;
	memcpy(p->payload, data, data_size);
	update_crc(p);

	return p;
}
WirelessPacket *clone_packet(WirelessPacket *p)
{
	WirelessPacket * p2 = (WirelessPacket*)malloc(p->length + PACKET_HEADER_SIZE);
	if (!p2)
		return NULL;

	memcpy(p2, p, p->length + PACKET_HEADER_SIZE);

	return p2;
}
void free_packet(WirelessPacket *p)
{
	free(p);
}
