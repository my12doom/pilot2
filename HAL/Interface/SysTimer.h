#pragma once

#include <stdint.h>

namespace HAL 
{
	class SysTimer
	{
	public:
		virtual int64_t gettime() = 0;		// micro-second
		virtual void delayms(int ms) = 0;
		virtual void delayus(int us) = 0;
	};
}
extern HAL::SysTimer *systimer;
