#pragma once

#include <stdint.h>

namespace HAL 
{
	enum interrupt_trigger
	{
		interrupt_rising = 1,
		interrupt_falling = 2,
		interrupt_rising_or_falling = 4,
	};

	typedef void (*interrupt_callback)(void *parameter, int flags);

	class IInterrupt
	{
	public:
		virtual void set_callback(interrupt_callback cb, void *parameter) = 0;
	};
}