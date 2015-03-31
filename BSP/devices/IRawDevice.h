#pragma once
#include <stdint.h>

namespace devices
{
	class IRawDevice
	{
	public:
		// return number of data read, negtive for error number
		virtual int read(void *out, int maxcount) = 0;

		// return number of data written, negtive for error number
		virtual int write(const void *out, int count) = 0;

		// device defined return value, negtive for error number
		virtual int ioctl(int request, void *data);

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}