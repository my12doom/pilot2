#pragma once
#include <stdint.h>

namespace HAL 
{
	class IADC
	{
	public:
		virtual int read() = 0;
	};
}