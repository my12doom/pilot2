#pragma once
#include <HAL/Interface/ISysTimer.h>
#include <stdint.h>

namespace STM32F0
{
	class F0SysTimer : public HAL::ISysTimer
	{
	public:
		F0SysTimer();
		~F0SysTimer(){}
		virtual int64_t gettime();		// micro-second
		virtual void delayms(int ms);
		virtual void delayus(int us);
	};
}