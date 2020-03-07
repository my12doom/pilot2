#pragma once

#include <stdint.h>

namespace HAL 
{
	class ISysTimer
	{
	public:
		virtual int64_t gettime() = 0;		// micro-second
		virtual void delaymsf(float ms){delayms(ms);}
		virtual void delayusf(float us){delayus(us);}
		virtual void delayms(int ms) = 0;
		virtual void delayus(int us) = 0;
		virtual int64_t rawtime(){return 0;}
	};
}
extern HAL::ISysTimer *systimer;
