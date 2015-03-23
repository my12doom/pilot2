#pragma once

#include <stdint.h>

namespace HAL
{
	class Storage
	{
		virtual int init() = 0;
		virtual int total_size() = 0;
		virtual int page_size() = 0;
		virtual int write(address, const void *data, int size) = 0;
		virtual int read(address, void *data, int maxsize) = 0;
	};
}
