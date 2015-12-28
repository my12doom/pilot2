#include "F4SysTimer.h"
#include <stm32f4xx.h>
#include <stdint.h>

namespace STM32F4 
{
	int64_t base = 0;
	int reload;
	
	extern "C" void SysTick_Handler()
	{
		base += reload;
	}
	
	F4SysTimer::F4SysTimer()
	{
		reload = SystemCoreClock / 100;		// ~ 10ms reload period
		SysTick_Config(reload);
		NVIC_SetPriority(SysTick_IRQn, 0x0);
	}
	
	
	int64_t F4SysTimer::gettime()		// micro-second
	{
		volatile int tick1 = SysTick->VAL;
		volatile int64_t tick_base1 = base;
		volatile int tick2 = SysTick->VAL;
		__DSB();
		__ISB();
		volatile int64_t tick_base2 = base;

		
		tick1 = reload - tick1;
		tick2 = reload - tick2;
		
		if (tick2 < tick1)
			return (tick_base2 + tick2) / (SystemCoreClock / 1000000);
		else
			return (tick_base1 + tick1) / (SystemCoreClock / 1000000);
	}
	
	void F4SysTimer::delayms(float ms)
	{
		delayus(ms*1000);
	}
	
	void F4SysTimer::delayus(float us)
	{
		static const float overhead = 0.37f;
		if (us < overhead)
			return;
		us -= overhead;
		if (us < 9000)	// ~ 9ms
		{
			volatile int start = reload - SysTick->VAL;
			volatile int target = int(start + us * (SystemCoreClock / 1000000)) % reload;

			if (start <= target)
			{
				while((reload - SysTick->VAL) < target && (reload - SysTick->VAL) >= start)
					;
			}
			else
			{
				while((reload - SysTick->VAL) >= start || (reload - SysTick->VAL) < target)
					;
			}
		}
		else
		{
			volatile int64_t t = gettime();
			while(gettime() - t < us)
				;
		}
	}
	
	static F4SysTimer timer;
}

HAL::ISysTimer *systimer = &STM32F4::timer;
