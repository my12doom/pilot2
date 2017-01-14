#pragma once

#include <stdint.h>

namespace HAL 
{
	typedef void (*timer_callback)(void *user_data);

	class ITimer
	{
	public:
		virtual void set_period(uint32_t period) = 0;				// micro-second
		virtual void set_callback(timer_callback cb, void *user_data = 0) = 0;
		virtual void restart(){}
		virtual void enable_cb(){}
		virtual void disable_cb(){}
	};
}