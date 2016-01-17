#include "SBusIN.h"
#include <string.h>

static int min(int a, int b)
{
	return a > b ? b : a;
}
static int max(int a, int b)
{
	return a > b ? a : b;
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
	int count = min(8 - start_channel, max_count);
	for(int i=0; i<count; i++)
	{
		switch (i+start_channel)
		{
		case 0:
			out[i] = last_frame.ch0;
			break;
		case 1:
			out[i] = last_frame.ch1;
			break;
		case 2:
			out[i] = last_frame.ch2;
			break;
		case 3:
			out[i] = last_frame.ch3;
			break;
		case 4:
			out[i] = last_frame.ch4;
			break;
		case 5:
			out[i] = last_frame.ch5;
			break;
		case 6:
			out[i] = last_frame.ch6;
			break;
		case 7:
			out[i] = last_frame.ch7;
			break;
		default:
			out[i] = 0;
		}
	}
	
	return count;
}

// return num channel written to out pointer
int sensors::SBusIN::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	int count = min(8 - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = last_packet_time;
	
	return count;
}

// statistics functions is mainly for RC calibration purpose.
int sensors::SBusIN::get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count)
{
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

void sensors::SBusIN::read_uart()
{
	// search startbytes
	while(port->available() > 0)
	{
		uint8_t s = 0;
		port->peak(&s, 1);

		if (s != 0x0f)
			port->read(&s, 1);
	}

	if (port->available() >= sizeof(sbus_frame))
	{
		port->read(&last_frame, sizeof(sbus_frame));

		if (last_frame.endbyte == 0x0b)
			last_packet_time = systimer->gettime();

		rc_static[0][0] = min(rc_static[0][0], last_frame.ch0);
		rc_static[0][1] = min(rc_static[0][1], last_frame.ch1);
		rc_static[0][2] = min(rc_static[0][2], last_frame.ch2);
		rc_static[0][3] = min(rc_static[0][3], last_frame.ch3);
		rc_static[0][4] = min(rc_static[0][4], last_frame.ch4);
		rc_static[0][5] = min(rc_static[0][5], last_frame.ch5);
		rc_static[0][6] = min(rc_static[0][6], last_frame.ch6);
		rc_static[0][7] = min(rc_static[0][7], last_frame.ch7);

		rc_static[1][0] = max(rc_static[1][0], last_frame.ch0);
		rc_static[1][1] = max(rc_static[1][1], last_frame.ch1);
		rc_static[1][2] = max(rc_static[1][2], last_frame.ch2);
		rc_static[1][3] = max(rc_static[1][3], last_frame.ch3);
		rc_static[1][4] = max(rc_static[1][4], last_frame.ch4);
		rc_static[1][5] = max(rc_static[1][5], last_frame.ch5);
		rc_static[1][6] = max(rc_static[1][6], last_frame.ch6);
		rc_static[1][7] = max(rc_static[1][7], last_frame.ch7);
	}
}

bool sensors::SBusIN::init(HAL::IUART *port)
{
	this->port = port;
	port->set_baudrate(100000);
	
	return true;
}