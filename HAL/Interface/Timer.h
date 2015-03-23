#pragma once

#include <stdint.h>

namespace HAL 
{
	class Timer
	{
	public:
		virtual void init() = 0;
		virtual void set_period() = 0;				// micro-second
		virtual void set_callback(int ms) = 0;		
	};
}