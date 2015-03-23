#pragma once

#include <stdint.h>

namespace HAL
{
	class UART
	{
	public:
		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int read(void *data, int max_count);
	};
}