#pragma once

#include <stdint.h>

#pragma pack(push, 1)

enum ZLinkManagement
{
	management_change_nodeid = 0,
	management_change_aes_key = 1,
	management_change_freq = 14,

	management_file_open = 2,
	management_file_block = 3,
	management_file_close = 4,
	management_file_delete = 5,
	management_file_play = 6,
	management_file_stop = 7,
	management_file_hash_request = 12,
	management_file_hash_result = 13,
	management_file_decompress_request = 15,
	management_file_decompress_result = 16,

	management_RGB = 8,
	management_reset_system = 9,

	management_port_power = 10,
	management_air_rate = 11,

	management_MAX = 17,
};

typedef struct
{
	uint8_t service;			// == management_change_nodeid
	uint16_t old_nodeid;
	uint16_t new_nodeid;
} change_nodeid_info;

typedef struct
{
	uint8_t service;			// == management_change_freq
	uint32_t uplink_freq;		// unit: hz
	uint32_t downlink_freq;		// unit: hz
} change_freq_info;

typedef struct
{
	uint8_t service;			// == management_change_aes_key
	uint8_t new_key[32];
} change_aes_key_info;

typedef struct
{
	uint8_t service;			// == management_file_open
	char filename[13];			// "8.3 filename", NULL terminated
} file_open_info;

typedef struct
{
	uint8_t service;			// == management_file_block
	uint8_t block_size;
	uint8_t block[];			// defined by block_size
} file_block_info;

typedef struct
{
	uint8_t service;			// == management_file_delete
	char filename[13];			// "8.3 filename", NULL terminated
} file_delete_info;

typedef struct
{
	uint8_t service;			// == management_file_play
	uint64_t ref_timestamp;		//	play starting ref timestamp
	char filename[13];			// "8.3 filename"
} file_play_info;

typedef struct
{
	uint8_t service;			// == management_file_hash_request
	char filename[13];			// "8.3 filename"
} file_hash_request_info;

typedef struct
{
	uint8_t service;			// == management_file_hash_result
	char filename[13];			// "8.3 filename"
	uint8_t sha1[20];
} file_hash_result_info;

typedef struct
{
	uint8_t service;			// == management_file_decompress_request
	char in_filename[13];		// "8.3 filename" of compressed file
	char out_filename[13];		// "8.3 filename" of output file
} file_decompress_request_info;

typedef struct
{
	uint8_t service;			// == management_file_decompress_result
	uint8_t success;			// 1 : success, 0: fail
	char out_filename[13];		// "8.3 filename" of output file
	uint8_t sha1[20];
} file_decompress_result_info;

typedef struct
{
	uint8_t service;			// == management_RGB
	uint8_t R;
	uint8_t G;
	uint8_t B;
} management_RGB_info;

typedef struct
{
	uint8_t service;			// == management_port_power
	uint8_t power;				// bit combination of port power, one bit for each uart port
								// uart_payload : 0x1
								// uart_RTK 	：0x2
								// RGB			: 0x4

} management_port_power_info;

typedef struct
{
	uint8_t service;			// == management_air_rate
	uint8_t rate;				// new air rate code
								// 0: lora SF7 500khz 4/5 CR
								// 1: GFSK 250kbps
} management_air_rate_info;



typedef union
{
	uint8_t					service;
	change_nodeid_info 		change_nodeid;
	change_aes_key_info		change_aes;
	change_freq_info		change_freq;
	file_open_info			file_open;
	file_block_info			file_block;
	file_delete_info		file_delete;
	file_play_info			file_play;
	file_hash_request_info	file_hash_request;
	file_hash_result_info	file_hash_result;
	file_decompress_request_info	file_decompress_request;
	file_decompress_result_info	file_decompress_result;
	management_RGB_info		RGB;
	management_port_power_info	uart_power;
	management_air_rate_info air_rate;
	
} management_info;

#pragma pack(pop)
