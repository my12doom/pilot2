#pragma once

#include <stdint.h>

namespace HAL 
{
	typedef void (*timer_callback)();

	class Timer
	{
	public:
		virtual void set_period() = 0;				// micro-second
		virtual void set_callback(timer_callback cb) = 0;		
	};
}