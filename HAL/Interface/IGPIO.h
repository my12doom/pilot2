#pragma once

#include <stdint.h>

namespace HAL 
{
	enum GPIO_MODE
	{
		MODE_IN = 0,
		MODE_OUT_PushPull = 1,
		MODE_OUT_OpenDrain = 2,
	};

	class IGPIO
	{
	public:
		virtual void set_mode(GPIO_MODE mode) = 0;
		virtual bool read() = 0;					// high = true, low = false
		virtual void write(bool newvalue) = 0;		// high = true, low = false
		virtual void toggle() = 0;
	};
}
