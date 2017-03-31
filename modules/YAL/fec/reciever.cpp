#include "reciever.h"
#include <stdio.h>
#include <string.h>
#include "oRS.h"
#include <stdlib.h>
#include <assert.h>

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
// 	packets = new raw_packet[270];
	memset(packets, 0, sizeof(raw_packet) * 256);
	current_frame_id = -1;
	current_packet_count = 0;
}

int reciever::put_packet(const void *packet, int size)
{
	// assume: minimum data error except packet loss
	//		   no out of order packets

	// reject ill conditioned packets
	if (size > sizeof(raw_packet))
		return -1;

	// decode header and check for error
	rsDecoder dec2;
	dec2.init(2);
	int g = dec2.correct_errors_erasures((unsigned char *)packet, HEADER_SIZE, 0, NULL);
	if (!g)
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
		assemble_and_out();

	return 0;
}

int reciever::assemble_and_out()
{
	if (current_frame_id < 0 || current_packet_count == 0)
		return 0;

	int payload_packet_count = 0;
	int parity_packet_count = 0;
	for(int i=0; i<256; i++)
	{
		if (packets[i].payload_packet_count)
		{
			payload_packet_count = packets[i].payload_packet_count;
			parity_packet_count = packets[i].parity_packet_count;
			break;
		}
	}

	// we have enough valid packets? packets properly formed?
	if (current_packet_count < payload_packet_count || payload_packet_count <= 0 || parity_packet_count <= 0)
	{
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
	

	if (payload_packet_count == 0 || parity_packet_count == 0 || payload_packet_count + parity_packet_count > 255)
		return -1;

	// assemble and do FEC if needed

	// create erasures
	int slice_size = payload_packet_count + parity_packet_count;
	int erasures[256];
	int erasures_count = 0;
	for(int i=0; i<slice_size; i++)
		if (packets[i].payload_packet_count == 0)	// current only missing packets checked, TODO: CRC
			erasures[erasures_count++] = i;

	// decode and output
	rsDecoder decoder;
	bool error = false;
	decoder.init(parity_packet_count);
	uint8_t slice_data[256];
	int max_packet_payload_size = sizeof(raw_packet)-HEADER_SIZE;
	frame * f = alloc_frame(payload_packet_count * max_packet_payload_size, current_frame_id, true);
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

	f->integrality = !error;
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