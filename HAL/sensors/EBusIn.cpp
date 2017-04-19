#include "EBusIn.h"
#include <string.h>
#include <Protocol/crc32.h>
static int min(int a, int b)
{
	return a > b ? b : a;
}
static int max(int a, int b)
{
	return a > b ? a : b;
}


sensors::EBusIN::EBusIN()
:last_packet_time(0)
{
}

// total channel count
int sensors::EBusIN::get_channel_count()
{
	return 6;
}

// return num channel written to out pointer
int sensors::EBusIN::get_channel_data(int16_t *out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(6 - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = last_frame.data[i+start_channel];	
	
	return count;
}

// return num channel written to out pointer
int sensors::EBusIN::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(6 - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = last_packet_time;
	
	return count;
}

// statistics functions is mainly for RC calibration purpose.
int sensors::EBusIN::get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count)
{
	read_uart();
	
	int count = min(6 - start_channel, max_count);
	memcpy(min_out, rc_static[0] + start_channel, count * sizeof(int16_t));
	memcpy(max_out, rc_static[1] + start_channel, count * sizeof(int16_t));
	
	return count;
}
int sensors::EBusIN::reset_statistics()
{
	int i;
	for(i=0; i<6; i++)
	{
		rc_static[0][i] = 32767;			// min
		rc_static[1][i] = 0;				// max
	}

	return 0;
}

HAL::RCIN_State sensors::EBusIN::state()
{
	if (systimer->gettime() > last_packet_time + 500000)
		return HAL::RCIN_Fail;

	//if (last_frame.flag & 0x8)
	//	return HAL::RCIN_Fail;

	return HAL::RCIN_Normal;
}

void sensors::EBusIN::read_uart()
{
	// search startbytes
	while(port->available() > 2)
	{
		uint8_t data[2] = {0};
		port->peak(&data, 2);

		if (data[0] != 0x85 || data[1] != 0xA3)
			port->read(data, data[1] == 0x85 ? 1 : 2);
		else
			break;
	}

	if (port->available() >= sizeof(last_frame))
	{
		ebus_data frame;
		port->read(&frame, sizeof(frame));
		if((uint8_t)crc32(0,frame.data, 12) == frame.crc32)
		{			
			last_frame = frame;
			last_packet_time = systimer->gettime();
			for(int i=0; i<6; i++)
				last_frame.data[i] = 1000 + last_frame.data[i] * 1000 /4096;
			
			//printf("EBUS:%d,%d,%d,%d,%d,%d %04x,%x\n", last_frame.data[0], last_frame.data[1], last_frame.data[2], last_frame.data[3],last_frame.data[4],last_frame.data[5], last_frame.startbytes,last_frame.crc32);
			for(int i=0; i<6; i++)
			{
				rc_static[0][i] = min(rc_static[0][i], last_frame.data[i]);
				rc_static[1][i] = max(rc_static[1][i], last_frame.data[i]);
			}
		}
		}
	}

bool sensors::EBusIN::init(HAL::IUART *port)
{
	this->port = port;
	port->set_baudrate(115200);
	
	return true;
}