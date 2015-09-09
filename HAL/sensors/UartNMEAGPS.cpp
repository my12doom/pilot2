#include "UartNMEAGPS.h"
#include <string.h>
#include <math.h>
#include <time.h>

namespace sensors
{

double NDEG2DEG(double ndeg)
{
	int degree = ndeg / 100;
	int minute = int(floor(ndeg)) % 100;	

	return degree + minute/60.0 + modf(ndeg, &ndeg)/60.0;
}

UartNMEAGPS::UartNMEAGPS()
{
	m_healthy = false;
	nmea_zero_INFO(&info);
	nmea_parser_init(&parser);
}

UartNMEAGPS::~UartNMEAGPS()
{
}


int UartNMEAGPS::init(HAL::IUART *uart, int baudrate)
{
	this->uart = uart;
	uart->set_baudrate(baudrate);
	return 0;
}

int UartNMEAGPS::read(devices::gps_data *data)
{
	int got = uart->readline(buffer+buffer_count, sizeof(buffer) - buffer_count);

	if (got > 0)
		m_healthy = true;
	else
		return 1;
	
	buffer_count += got;

	if (!m_healthy)
		return -1;

	int return_pos = -1;
	for(int i = buffer_count - 1; i>0; i--)
	{
		if (buffer[i] == '\n')
		{
			return_pos = i+1;
			break;
		}
	}

	int return_value = 1;
	if (return_pos > 0)
	{
		int parse_result = nmea_parse(&parser, buffer, return_pos, &info);
		memmove(buffer, buffer+return_pos, buffer_count - return_pos);
		buffer_count -= return_pos;

		return_value = parse_result > 0 ? 0 : 1;
	}
	
	// copy from info
	data->longitude = NDEG2DEG(info.lon);		// longitude in degree
	data->latitude = NDEG2DEG(info.lat);		// latitude in degree
	data->speed = info.speed / 3.6f;			// unit: meter/s
	data->altitude = info.elv;					// meter
	data->direction = info.direction;			// Track angle in degrees True, 0-360 degree, 0: north, 90: east, 180: south, 270: west, 359:almost north
	data->DOP[0] = info.PDOP*100;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	data->DOP[1] = info.HDOP*100;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	data->DOP[2] = info.VDOP*100;				// DOP[3]: PDOP, HDOP, VOP, unit base: 0.01
	data->satelite_in_view = info.satinfo.inview;
	data->satelite_in_use = info.satinfo.inuse;
	data->sig = info.sig;						// GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive)
	data->fix = info.fix;						// Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D)
	data->declination = info.declination * 100.0f;// Magnetic variation in 0.01 degrees (Easterly var. subtracts from true course)

	struct tm _tm;	
	_tm.tm_sec = info.utc2.sec;
	_tm.tm_min = info.utc2.min;
	_tm.tm_hour = info.utc2.hour;
	_tm.tm_mday = info.utc2.day;
	_tm.tm_mon = info.utc2.mon;
	_tm.tm_year = info.utc2.year;	
	
	data->timestamp = mktime(&_tm);
	data->timestamp += info.utc2.zone_hour * 3600;
	
	return return_value;
}

bool UartNMEAGPS::healthy()
{
	if (!m_healthy)
	{
		devices::gps_data tmp;
		read(&tmp);
	}
	
	return m_healthy;
}

}