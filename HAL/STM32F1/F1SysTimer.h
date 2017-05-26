#pragma once
#include <HAL/Interface/ISysTimer.h>
#include <stdint.h>

namespace STM32F1
{
	class F1SysTimer : public HAL::ISysTimer
	{
	public:
		F1SysTimer();
		~F1SysTimer(){}
		virtual int64_t gettime();		// micro-second
		virtual void delayms(int ms);
		virtual void delayus(int us);
	};
}
