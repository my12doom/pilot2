#pragma once

#include <stdint.h>

namespace HAL
{
	class UART
	{
		public:
		virtual int set_baudrate(int baudrate) = 0;
		virtual int write(const void *data, int count) =0;
		virtual int read(void *data, int max_count)=0;
	};
}