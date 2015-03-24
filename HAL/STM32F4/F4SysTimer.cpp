#pragma once
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
		reload = SystemCoreClock / 100;
		SysTick_Config(reload);
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
	
	void F4SysTimer::delayms(int ms)
	{
	}
	
	void F4SysTimer::delayus(int us)
	{
	}
	
	static F4SysTimer timer;
}

HAL::SysTimer *systimer = &STM32F4::timer;
