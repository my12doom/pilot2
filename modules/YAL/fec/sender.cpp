#include "sender.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "frame.h"

FrameSender::FrameSender()
{
	frame_id = 0;
	block_sender = NULL;
	packets = new raw_packet[256];
	config(PACKET_SIZE, 1.5);
}

FrameSender::~FrameSender()
{
	delete [] packets;

}

int FrameSender::set_block_device(HAL::IBlockDevice *block_sender)
{
	this->block_sender = block_sender;

	return 0;
}

int FrameSender::config(int packet_size, float parity_ratio)
{
	this->max_packet_payload_size = packet_size-HEADER_SIZE;
	this->parity_ratio = parity_ratio;

	return 0;
}

int FrameSender::send_frame(const void *payload, int payload_size)
{
	int payload_packet_count = (payload_size + max_packet_payload_size - 1) / max_packet_payload_size;
	int parity_packet_count = ceil(payload_packet_count * parity_ratio);
	if (parity_packet_count > MAX_NPAR)
		parity_packet_count = MAX_NPAR;
	int slice_size = payload_packet_count + parity_packet_count;

	if (slice_size > 255)		// too large frame
	{
		printf("too large frame\n");
		return -1;
	}

	rsEncoder encoder;
	encoder.init(parity_packet_count);
	printf("sending %d bytes frame, %d+%d packets\n", payload_size, payload_packet_count, parity_packet_count);

	memset(packets, 0, (slice_size) * sizeof(raw_packet));

	for(int i=0; i<max_packet_payload_size; i++)
	{
		uint8_t slice_data[256];
		memset(slice_data, 0, slice_size);
		
		for(int j=0; j<payload_packet_count; j++)
		{
			int pos = j*max_packet_payload_size + i;
			if (pos >= payload_size)
				slice_data[j] = 0;
			else
				slice_data[j] = ((uint8_t*)payload)[pos];
		}

		encoder.resetData();
		encoder.append_data(slice_data, payload_packet_count);
		encoder.output(slice_data+payload_packet_count);

		for(int j=0; j<slice_size; j++)
			packets[j].data[i] = slice_data[j];
	}

	// create header rs parity and send out
	rsEncoder header_rs_encoder;
	header_rs_encoder.init(sizeof(packets->header_rs));
	for(int i=0; i<slice_size; i++)
	{
		packets[i].frame_id = frame_id;
		packets[i].last_frame_packet_count = 0;
		packets[i].packet_id = i;
		packets[i].parity_packet_count = parity_packet_count;
		packets[i].payload_packet_count = payload_packet_count;

		header_rs_encoder.resetData();
		header_rs_encoder.append_data((unsigned char*)&packets[i], HEADER_SIZE-sizeof(packets[i].header_rs));
		header_rs_encoder.output(&packets[i].header_rs[0]);

		send_packet(&packets[i], sizeof(raw_packet));
	}

	frame_id++;

	return 0;
}

int FrameSender::send_packet(const void *payload, int payload_size)
{
	if (block_sender)
		block_sender->write(payload, payload_size);

	return 0;
}