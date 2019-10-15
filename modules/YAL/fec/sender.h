#pragma once

#include <stdint.h>
#include "oRS.h"
#include "frame.h"
#include <HAL/Interface/IBlockDevice.h>

class FrameSender
{
public:
	FrameSender();
	virtual ~FrameSender();

	virtual int set_block_device(HAL::IBlockDevice *block_sender);
	virtual int config(int packet_size, float residual_ratio, int min_parity_packet_count = 20);
	virtual int send_frame(const void *payload, int payload_size);
	virtual int send_packet(const void *payload, int payload_size);
protected:

	int packet_payload_size;
	float parity_ratio;

	raw_packet *packets;
	uint8_t frame_id;
	HAL::IBlockDevice *block_sender;

	int min_parity_packet_count;

	uint8_t *tmp;// [256*MAX_PAYLOAD_SIZE];
};