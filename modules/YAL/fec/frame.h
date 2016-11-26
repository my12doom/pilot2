#pragma once

#include <stdint.h>

#define PACKET_SIZE 1400

// frame structure for exchange data in program.
typedef struct frame_struct
{
	int frame_id;
	bool integrality;
	int payload_size;
	void *payload;
} frame;

// packet structure for IO
typedef struct raw_packet_struct
{
	uint8_t frame_id;
	uint8_t packet_id;
	uint8_t payload_packet_count;		// 0 means invalid packet
	uint8_t parity_packet_count;
	uint8_t last_frame_packet_count;
	uint8_t data[PACKET_SIZE-5];
} raw_packet;

// interface
class IFrameReciever
{
public:
	virtual ~IFrameReciever(){};
	virtual int handle_event() = 0;
	virtual int handle_frame(const frame &frame) = 0;
};

// helper function
void release_frame(frame *f);
frame *alloc_frame(int payload_size, int frame_id = 0, bool integrality = false);
frame *clone_frame(const frame *f);