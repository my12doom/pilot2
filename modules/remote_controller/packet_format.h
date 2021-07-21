#pragma once

#include <stdint.h>

#pragma pack(push, 1)

typedef struct
{
	uint16_t hoop_id;
	uint8_t payload[28];
	uint16_t crc;
} nrf_pkt;

typedef struct
{
	int16_t channel_data[6];
	uint8_t keys[2];
	uint8_t ack_id : 4;
	uint8_t telemetry_id : 4;
	uint8_t telemetry_size;
	uint8_t telemetry[12];
} uplink_payload_v0;

typedef struct
{
	uint8_t ack_id : 4;
	uint8_t telemetry_id : 4;
	uint8_t telemetry_size;
	uint8_t telemetry[26];
} downlink_payload_v0;

#pragma pack(pop)
