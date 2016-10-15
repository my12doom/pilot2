#pragma once

#include <stdint.h>

namespace HAL
{
	class IUART
	{
		public:
		virtual int set_baudrate(int baudrate) = 0;
		virtual int peak(void *data, int max_count) = 0;
		virtual int write(const void *data, int count) =0;
		virtual int flush() = 0;
		virtual int read(void *data, int max_count)=0;
		virtual int readline(void *data, int max_count)=0;
		virtual int set_rx_calback(){return -1;}		// default: return -1 = not implemented
		virtual int available() =0;
	};
}
