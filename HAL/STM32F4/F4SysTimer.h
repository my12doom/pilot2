#pragma once
#include "SysTimer.h"

#include <stdint.h>

namespace STM32F4 
{
	class F4SysTimer : public HAL::SysTimer
	{
	public:
		F4SysTimer();
		~F4SysTimer(){}
		virtual int64_t gettime();		// micro-second
		virtual void delayms(int ms);
		virtual void delayus(int us);
	};
}