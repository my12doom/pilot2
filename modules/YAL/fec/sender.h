#pragma once

#include <stdint.h>
#include "oRS.h"
#include "frame.h"

class FrameSender
{
public:
	FrameSender();
	virtual ~FrameSender();

	virtual int config(int packet_size, float residual_ratio);
	virtual int send_frame(const void *payload, int payload_size);
	virtual int send_packet(const void *payload, int payload_size);
protected:

	int max_packet_payload_size;
	float parity_ratio;
	rsEncoder encoder;

	raw_packet *packets;
	uint8_t frame_id;
};