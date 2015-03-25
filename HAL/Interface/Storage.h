#pragma once

#include <stdint.h>

namespace HAL
{
	class Storage
	{
	public:
		virtual int init() = 0;
		virtual int total_size() = 0;
		virtual int page_size() = 0;
		virtual int erase(int address)=0;
		virtual int write(int address, const void *data, int size) = 0;
		virtual int read(int address, void *data, int maxsize) = 0;
	};
}
