#pragma once
#include <stdint.h>

#define ZLinkMaxFileBlockPayload 512

#pragma pack(push, 1)


typedef struct
{
	uint16_t magic;		// big-endian 0x85A3, little endian 0xA385
	uint16_t crc;
	uint8_t size;
	uint8_t type;
	int64_t timestamp;	// time stamp in microseconds
	uint8_t payload[0];
} ZLinkFileBlock;

enum ZLinkFileBlockType
{
	block_uart_payload = 0,		// uint8_t[length] payload
	block_RGB_cmd = 1,			// uint8_t[3] = {R, G, B}
	block_mavlink_ned = 2,		// ned_block_t * (in mavlink.h)
};

#pragma pack(pop)


void update_crc(ZLinkFileBlock *b);
bool check_crc(const ZLinkFileBlock *b);