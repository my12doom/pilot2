#pragma once

#include <stdint.h>

namespace HAL 
{
	typedef void (*timer_callback)();

	class ITimer
	{
	public:
		virtual void set_period(uint32_t period) = 0;				// micro-second
		virtual void set_callback(timer_callback cb) = 0;
		virtual void call_callback() = 0;	
	};
}