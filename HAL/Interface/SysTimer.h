#pragma once

#include <stdint.h>

namespace HAL 
{
	class SysTimer
	{
	public:
		virtual void init() = 0;
		virtual int64_t gettime() = 0;		// micro-second
		virtual void delayms(int ms) = 0;
		virtual void delayus(int us) = 0;
	};
}