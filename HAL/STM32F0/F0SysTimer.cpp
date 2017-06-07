#include "F0SysTimer.h"
#include <stm32f0xx.h>
#include <stdint.h>

namespace STM32F0 
{
	int64_t base = 0;
	
	extern "C" void TIM6_DAC_IRQHandler()
	{
		TIM_ClearITPendingBit(TIM6 , TIM_FLAG_Update);
		base += 0x10000;
	}
	
	F0SysTimer::F0SysTimer()
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIM6);
		TIM_InternalClockConfig(TIM6);
		SystemCoreClockUpdate();
		TIM_TimeBaseStructure.TIM_Prescaler= SystemCoreClock / 1000000-1;
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period=0xffff;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
		TIM_TimeBaseInit(TIM6,&TIM_TimeBaseStructure);
		TIM_ClearFlag(TIM6,TIM_FLAG_Update);
		TIM_ARRPreloadConfig(TIM6,DISABLE);
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIM6,ENABLE);
	}
	
	
	int64_t F0SysTimer::gettime()		// micro-second
	{
		volatile int tick1 = TIM6->CNT;
		volatile int64_t tick_base1 = base;
		volatile int tick2 = TIM6->CNT;
		__DSB();
		__ISB();
		volatile int64_t tick_base2 = base;
				
		if (tick2 < tick1)
			return (tick_base2 + tick2);
		else
			return (tick_base1 + tick1);
	}
	
	void F0SysTimer::delayms(int ms)
	{
		delayus(ms*1000);
	}
	
	void F0SysTimer::delayus(int us)
	{
		volatile int start = TIM6->CNT;
				
		if (us < 30000)
		{
			volatile int target = int(start + us) & 0xffff;

			if (start <= target)
			{
				while((TIM6->CNT) < target && (TIM6->CNT) >= start)
					;
			}
			else
			{
				while((TIM6->CNT) >= start || (TIM6->CNT) < target)
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
	
	static F0SysTimer timer;
}

HAL::ISysTimer *systimer = &STM32F0::timer;
