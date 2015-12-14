#pragma once
#include <HAL/Interface/ISysTimer.h>
#include <stdint.h>

namespace STM32F7_HAL
{
	class F7SysTimer : public HAL::ISysTimer
	{
	public:
		F7SysTimer();
		~F7SysTimer(){}
		virtual int64_t gettime();		// micro-second
		virtual void delayms(float ms);
		virtual void delayus(float us);
		void config();
	};
}