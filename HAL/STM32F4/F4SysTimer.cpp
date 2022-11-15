#include "F4SysTimer.h"
#include <stm32f4xx.h>
#include <stm32f4xx_tim.h>
#include <stdint.h>

namespace STM32F4 
{
	int64_t base = 0;
	
	extern "C" void TIM1_BRK_TIM9_IRQHandler()
	{
		if (TIM9->SR & TIM_IT_Update)
		{
			base += 0x10000;
			TIM_ClearFlag(TIM9, TIM_IT_Update);	
		}
	}
	
	F4SysTimer::F4SysTimer()
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		
		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
		TIM_TimeBaseStructure.TIM_Period = 0xffff;
		TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock/1000000)-1;
		TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM9, &TIM_TimeBaseStructure);
		
		TIM_Cmd(TIM9, ENABLE);
		
		TIM_ITConfig(TIM9, TIM_IT_Update, ENABLE);
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);

	}
	
	
	int64_t F4SysTimer::gettime()		// micro-second
	{
		volatile int tick1 = TIM9->CNT;
		volatile int64_t tick_base1 = base;
		volatile int tick2 = TIM9->CNT;
		__DSB();
		__ISB();
		volatile int64_t tick_base2 = base;
				
		if (tick2 < tick1)
			return (tick_base2 + tick2);
		else
			return (tick_base1 + tick1);
	}
	
	void F4SysTimer::delayms(int ms)
	{
		delayus(ms*1000);
	}
	
	void F4SysTimer::delayus(int us)
	{
		volatile int start = TIM9->CNT;
				
		if (us < 30000)
		{
			volatile int target = int(start + us) & 0xffff;

			if (start <= target)
			{
				while((TIM9->CNT) < target && (TIM9->CNT) >= start)
					;
			}
			else
			{
				while((TIM9->CNT) >= start || (TIM9->CNT) < target)
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
