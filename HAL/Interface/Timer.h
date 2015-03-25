#pragma once

#include <stdint.h>

namespace HAL 
{
	class Timer
	{
	public:
		virtual void set_period(uint32_t period) = 0;				// micro-second
		virtual void set_callback(int ms) = 0;		
	};
}