#include "SBusIN.h"
#include <string.h>

struct sbus_bit_pick {
	uint8_t byte;
	uint8_t rshift;
	uint8_t mask;
	uint8_t lshift;
};
static const struct sbus_bit_pick sbus_decoder[16][3] = {
	/*  0 */ { { 0, 0, 0xff, 0}, { 1, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  1 */ { { 1, 3, 0x1f, 0}, { 2, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/*  2 */ { { 2, 6, 0x03, 0}, { 3, 0, 0xff, 2}, { 4, 0, 0x01, 10} },
	/*  3 */ { { 4, 1, 0x7f, 0}, { 5, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/*  4 */ { { 5, 4, 0x0f, 0}, { 6, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/*  5 */ { { 6, 7, 0x01, 0}, { 7, 0, 0xff, 1}, { 8, 0, 0x03,  9} },
	/*  6 */ { { 8, 2, 0x3f, 0}, { 9, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/*  7 */ { { 9, 5, 0x07, 0}, {10, 0, 0xff, 3}, { 0, 0, 0x00,  0} },
	/*  8 */ { {11, 0, 0xff, 0}, {12, 0, 0x07, 8}, { 0, 0, 0x00,  0} },
	/*  9 */ { {12, 3, 0x1f, 0}, {13, 0, 0x3f, 5}, { 0, 0, 0x00,  0} },
	/* 10 */ { {13, 6, 0x03, 0}, {14, 0, 0xff, 2}, {15, 0, 0x01, 10} },
	/* 11 */ { {15, 1, 0x7f, 0}, {16, 0, 0x0f, 7}, { 0, 0, 0x00,  0} },
	/* 12 */ { {16, 4, 0x0f, 0}, {17, 0, 0x7f, 4}, { 0, 0, 0x00,  0} },
	/* 13 */ { {17, 7, 0x01, 0}, {18, 0, 0xff, 1}, {19, 0, 0x03,  9} },
	/* 14 */ { {19, 2, 0x3f, 0}, {20, 0, 0x1f, 6}, { 0, 0, 0x00,  0} },
	/* 15 */ { {20, 5, 0x07, 0}, {21, 0, 0xff, 3}, { 0, 0, 0x00,  0} }
};

#define SBUS_FRAME_SIZE 25


static int min(int a, int b)
{
	return a > b ? b : a;
}
static int max(int a, int b)
{
	return a > b ? a : b;
}

int sbus2pwm(int sbus)
{
	return 1000+sbus*1000/2048;
}

int sbus_decode(uint8_t *frame_data, sbus_data *decoded_data)
{
	if (frame_data[0] != 0x0f)
		return -1;

	if (frame_data[24] != 0x00 && frame_data[24] != 0x03 && frame_data[24] != 0x83 && frame_data[24] != 0x43 && frame_data[24] != 0xC3 && frame_data[24] != 0x23 && frame_data[24] != 0xA3 && frame_data[24] != 0x63 && frame_data[24] != 0xE3)
		return -1;

	for (int channel = 0; channel < 16; channel++) {
		unsigned value = 0;

		for (unsigned pick = 0; pick < 3; pick++) {
			const struct sbus_bit_pick *decode = &sbus_decoder[channel][pick];

			if (decode->mask != 0) {
				unsigned piece = frame_data[1 + decode->byte];
				piece >>= decode->rshift;
				piece &= decode->mask;
				piece <<= decode->lshift;

				value |= piece;
			}
		}

		/* convert 0-2048 values to 1000-2000 ppm encoding in a not too sloppy fashion */
		decoded_data->data[channel] = 1000 + value * 1000 / 2048;
	}

	return 0;
}

sensors::SBusIN::SBusIN()
:last_packet_time(0)
{
}

// total channel count
int sensors::SBusIN::get_channel_count()
{
	return 8;
}

// return num channel written to out pointer
int sensors::SBusIN::get_channel_data(int16_t *out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(8 - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = last_frame.data[i+start_channel];	
	
	return count;
}

// return num channel written to out pointer
int sensors::SBusIN::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(8 - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = last_packet_time;
	
	return count;
}

// statistics functions is mainly for RC calibration purpose.
int sensors::SBusIN::get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(8 - start_channel, max_count);
	memcpy(min_out, rc_static[0] + start_channel, count * sizeof(int16_t));
	memcpy(max_out, rc_static[1] + start_channel, count * sizeof(int16_t));
	
	return count;
}
int sensors::SBusIN::reset_statistics()
{
	int i;
	for(i=0; i<8; i++)
	{
		rc_static[0][i] = 32767;			// min
		rc_static[1][i] = 0;				// max
	}

	return 0;
}

HAL::RCIN_State sensors::SBusIN::state()
{
	if (systimer->gettime() > last_packet_time + 500000)
		return HAL::RCIN_Fail;

	if (last_frame.flag & 0x8)
		return HAL::RCIN_Fail;

	return HAL::RCIN_Normal;
}

void sensors::SBusIN::read_uart()
{
	// search startbytes
	while(port->available() > 0)
	{
		uint8_t s = 0;
		port->peak(&s, 1);

		if (s != 0x0f)
			port->read(&s, 1);
		else
			break;
	}

	if (port->available() >= SBUS_FRAME_SIZE)
	{
		uint8_t tmp[SBUS_FRAME_SIZE] = {0};
		port->read(tmp, sizeof(tmp));
		if (sbus_decode(tmp, &last_frame) == 0)
			last_packet_time = systimer->gettime();

		for(int i=0; i<8; i++)
		{
			rc_static[0][i] = min(rc_static[0][i], last_frame.data[i]);
			rc_static[1][i] = max(rc_static[1][i], last_frame.data[i]);
		}
	}
}

bool sensors::SBusIN::init(HAL::IUART *port)
{
	this->port = port;
	port->set_baudrate(100000);
	
	return true;
}