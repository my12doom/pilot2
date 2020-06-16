#pragma once
#include <stdint.h>

typedef struct 
{
	uint16_t magic;
	uint16_t version;
	uint32_t size;
	uint32_t compressed_size;
	uint32_t crc;
	uint32_t compressed_crc;
	uint8_t padding[512-20];
}LZO_HEADER;

int decompress_file(const char *src, const char *dst);
