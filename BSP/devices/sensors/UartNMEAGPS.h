#pragma once

#include <stdint.h>
#include <Interfaces.h>
#include <BSP/devices/IGPS.h>
#include <NMEA/nmea.h>

using namespace HAL;
namespace sensors
{
	class UartNMEAGPS : public devices::IGPS
	{
	public:
		UartNMEAGPS();
		~UartNMEAGPS();

		int init(HAL::IUART *uart, int baudrate);
		virtual int read(devices::gps_data *data);
		virtual bool healthy();
	protected:
		HAL::IUART *uart;
		char buffer[1024];
		int buffer_count;
		nmeaINFO info;
		nmeaPARSER parser;
		bool m_healthy;
	};
}
