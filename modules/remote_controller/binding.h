/*
binding workflow

tx side:
1. send all binding request packets out.
2. switch to rx mode for 50ms, listen for ACK packets, if ACK packets received, exit binding loop
3. repeat step 1-2.

rx side:
1. listen for both binding request packets and normal tx packets.
2. repeat step 1 until: valid and supported packets recieved.
						or normal tx packets recieved.
						or timed out ( if any ).
3. if binding packets recieved, enter tx mode, send ACK packets for at least 200ms.
4. save settings and exit binding loop.


possible errors:
1. RX recieved binding packets and exit binding loop, but failed to ACK tx:
	binding is done and should work properly.
	reset tx to exit binding.
	or reset rx to bind again.
*/


#pragma once

#include <stdint.h>

static const int BINDING_CHANNEL = 15;
static uint8_t BINDING_ADDRESS[3] = {0x85, 0xA3, 0x00};

typedef struct _binding_pkt
{
	uint8_t magic[2];		// {'B', 'D'} for "binding"
	uint8_t crc;			// lower byte of crc32 of the remainning 29 bytes
	uint8_t version;
	uint8_t payload[28];
} binding_pkt;

typedef struct _binding_info_v0
{
	uint8_t cmd;
	uint8_t key[8];			// key for both hooping randomizer and any encryption.
	uint8_t channel_high;
	uint8_t channel_low;
} binding_info_v0;

enum cmd_v0
{
	cmd_requesting_binding = 0,
	cmd_ack_binding = 1,
	cmd_nack_binding = 2,
	cmd_done_binding = 3,
};
