#include "reciever.h"
#include <stdio.h>
#include <string.h>
#include "oRS.h"
#include <stdlib.h>
#include <assert.h>
#include "cauchy_256.h"
#include <Protocol/crc32.h>

reciever::reciever()
{
	cb = NULL;
	init();
}

reciever::reciever(IFrameReciever *cb)
:cb(cb)
{
	init();
}

reciever::~reciever()
{
// 	delete [] packets;
}

void reciever::init()
{
	cauchy_256_init();
// 	packets = new raw_packet[270];
	memset(packets, 0, sizeof(raw_packet) * 256);
	current_frame_id = -1;
	current_packet_count = 0;
}

int reciever::put_packet(const void *packet, int size)
{
	// assume: no out of order packets

	// reject ill conditioned packets
	if (size > sizeof(raw_packet) || size < HEADER_SIZE)
		return -1;

	// decode header and check for error
	uint32_t crc_calculated = crc32(0, packet, HEADER_SIZE - sizeof(raw_packet::header_crc));
	raw_packet *raw = (raw_packet*)packet;
	uint8_t crc0 = crc_calculated;
	uint8_t crc1 = crc_calculated >> 8;
	if (crc0 != raw->header_crc[0] || crc1 != raw->header_crc[1])
		return -2; 

	// check for new frame
	raw_packet *p = (raw_packet*)packet;
	if (p->frame_id != current_frame_id)
		assemble_and_out();

	// place packet in buffer
	if (packets[p->packet_id].payload_packet_count)
		printf("warning: possible duplicated packet %d\n", p->packet_id);

 	//printf("rx:packet #%d of frame #%d, (%d+%d packets)\n", p->packet_id, p->frame_id, p->payload_packet_count, p->parity_packet_count);

	memcpy(&packets[p->packet_id], packet, size);
	current_packet_count ++;
	current_frame_id = p->frame_id;

	// last packet?
	if (p->packet_id == p->payload_packet_count + p->parity_packet_count - 1)
	{
		assemble_and_out();
	}

	return 0;
}

int reciever::assemble_and_out()
{
	if (current_frame_id < 0 || current_packet_count == 0)
		return 0;

	int payload_packet_count = 0;
	int parity_packet_count = 0;
	int frame_id = 0;

	for(int i=0; i<256; i++)
	{
		if (packets[i].payload_packet_count)
		{
			payload_packet_count = packets[i].payload_packet_count;
			parity_packet_count = packets[i].parity_packet_count;
			frame_id = packets[i].frame_id;
			break;
		}
	}

	//printf("assemble_and_out %d \n", frame_id);

	// we have enough valid packets? packets properly formed?
	if (current_packet_count < payload_packet_count || payload_packet_count <= 0 || parity_packet_count <= 0)
	{
		if (current_packet_count)
		printf("not enough packets, %d/%d\n", current_packet_count, payload_packet_count);


		// no, clear up and exit
		current_frame_id = -1;
		current_packet_count = 0;
		for(int i=0; i<256; i++)
		{
			memset(&packets[i], 0, sizeof(raw_packet));
			packets[i].payload_packet_count = 0;
		}


		return -1;
	}

	//printf("assemble: %d/%d packets\n", current_packet_count, payload_packet_count);
	

	if (payload_packet_count == 0 || parity_packet_count == 0 || payload_packet_count + parity_packet_count > 255)
		return -1;

	// assemble and do FEC
	int slice_size = payload_packet_count + parity_packet_count;
	int max_packet_payload_size = sizeof(raw_packet)-HEADER_SIZE;
	frame * f = alloc_frame(payload_packet_count * max_packet_payload_size, current_frame_id, true);
	bool error = false;

#if !USE_CAUCHY
	// create erasures
	int erasures[256];
	int erasures_count = 0;
	for(int i=0; i<slice_size; i++)
		if (packets[i].payload_packet_count == 0)	// current only missing packets checked, TODO: CRC
			erasures[erasures_count++] = i;

	rsDecoder decoder;
	decoder.init(parity_packet_count);
	uint8_t slice_data[256];
	for(int i=0; i<max_packet_payload_size; i++)
	{
		for(int j=0; j<slice_size; j++)
			slice_data[j] = packets[j].data[i];		

		if (!decoder.correct_errors_erasures(slice_data, slice_size, erasures_count, erasures))
			error = true;


		for(int j=0; j<payload_packet_count; j++)
		{
			((uint8_t*)f->payload)[j*max_packet_payload_size + i] = slice_data[j];			
		}
	}
#else
	Block blocks[256] = {0};
	int j = 0;
	for(int i=0; i<slice_size; i++)
	{
		if (packets[i].payload_packet_count)
		{
			blocks[j].data = packets[i].data;
			blocks[j].row = i;
			j++;
		}
	}

	assert(j>=payload_packet_count);

	error = cauchy_256_decode(payload_packet_count, parity_packet_count, blocks, payload_packet_count, max_packet_payload_size);

// 	for(int i=0; i<payload_packet_count; i++)
// 		assert(blocks[i].row == i);

	int copied = 0;

	for(int i=0; i<j; i++)
	{
		if (blocks[i].row < payload_packet_count && blocks[i].data)
		{
			memcpy((uint8_t*)f->payload+blocks[i].row*max_packet_payload_size, blocks[i].data, max_packet_payload_size);
			copied ++;
		}
	}

	if (copied != payload_packet_count)
		error = true;
#endif

	f->integrality = !error;
	uint32_t frame_data_size = *(int*)((uint8_t*)f->payload+4);
	uint32_t crc = *(uint32_t*)f->payload;
	if (frame_data_size <= f->payload_size-8 && crc == crc32(0, (uint8_t*)f->payload+4, frame_data_size+4))
		memmove(f->payload, (uint8_t*)f->payload+4, frame_data_size+4);
	else
		f->integrality = false;

	if (cb)
		cb->handle_frame(*f);
	release_frame(f);

	// clear up
	current_frame_id = -1;
	current_packet_count = 0;
	for(int i=0; i<256; i++)
	{
		packets[i].payload_packet_count = 0;
		memset(&packets[i], 0, sizeof(raw_packet));
	}

	return 0;
}
