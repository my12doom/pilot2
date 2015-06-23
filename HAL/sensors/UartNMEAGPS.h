#pragma once

#include <stdint.h>
#include <Interfaces.h>
#include <HAL/Interface/IGPS.h>
#include <NMEA/nmea.h>

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
		char black_magic[1024];			// magic buffer to fix unknown buffer overrun in nmea parser library.
		bool m_healthy;
	};
}
