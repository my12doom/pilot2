#pragma once

#include <stdint.h>

namespace HAL 
{
	class ADC
	{
	public:
		virtual int read() = 0;
	};
}