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

	class GPIO
	{
	public:
		virtual void set_mode(GPIO_MODE mode) = 0;			
		virtual bool read() = 0;
		virtual void write(bool new value) = 0;
		virtual void toggle() = 0;
	};
}