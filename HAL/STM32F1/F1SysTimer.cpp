#include "F1SysTimer.h"
#include <stm32f10x.h>
#include <stm32f10x_tim.h>
#include <misc.h>
#include <stdint.h>

namespace STM32F1 
{
	int64_t base = 0;
	
	extern "C" void TIM1_UP_IRQHandler()
	{
		TIM_ClearITPendingBit(TIM1 , TIM_FLAG_Update);
		base += 0x10000;
	}
	
	F1SysTimer::F1SysTimer()
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIM1);
		TIM_InternalClockConfig(TIM1);
		SystemCoreClockUpdate();
		TIM_TimeBaseStructure.TIM_Prescaler= SystemCoreClock / 1000000-1;
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period=0xffff;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
		TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);
		TIM_ClearFlag(TIM1,TIM_FLAG_Update);
		TIM_ARRPreloadConfig(TIM1,DISABLE);
		TIM_ITConfig(TIM1,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIM1,ENABLE);
	}
	
	
	int64_t F1SysTimer::gettime()		// micro-second
	{
		volatile int tick1 = TIM1->CNT;
		volatile int64_t tick_base1 = base;
		volatile int tick2 = TIM1->CNT;
		__DSB();
		__ISB();
		volatile int64_t tick_base2 = base;
				
		if (tick2 < tick1)
			return (tick_base2 + tick2);
		else
			return (tick_base1 + tick1);
	}
	
	void F1SysTimer::delayms(int ms)
	{
		delayus(ms*1000);
	}
	
	void F1SysTimer::delayus(int us)
	{
		volatile int start = TIM1->CNT;
				
		if (us < 30000)
		{
			volatile int target = int(start + us) & 0xffff;

			if (start <= target)
			{
				while((TIM1->CNT) < target && (TIM1->CNT) >= start)
					;
			}
			else
			{
				while((TIM1->CNT) >= start || (TIM1->CNT) < target)
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
	
	static F1SysTimer timer;
}

HAL::ISysTimer *systimer = &STM32F1::timer;
