#pragma once

#include <stdint.h>

namespace HAL 
{
	class ISysTimer
	{
	public:
		virtual int64_t gettime() = 0;		// micro-second
		virtual void delayms(float ms) = 0;
		virtual void delayus(float us) = 0;
	};
}
extern HAL::ISysTimer *systimer;
