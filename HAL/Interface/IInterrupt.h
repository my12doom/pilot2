#pragma once

#include <stdint.h>

namespace HAL 
{
	typedef void (*interrupt_callback)(int parameter);

	class IInterrupt
	{
	public:
		virtual void set_callback(interrupt_callback cb) = 0;
	};
}