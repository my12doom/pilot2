#pragma once

#include <stdint.h>
#include "ITimer.h"
#include "stm32f4xx.h"
using namespace HAL;
namespace STM32F4
{
	class F4Timer:public ITimer
	{
	private:
		timer_callback cb;
		TIM_TypeDef* TIMx;
		void TimerInit(TIM_TypeDef* TIMx);
		void TIM1_UP_TIM10_IRQHandler(void);
		void TIM2_IRQHandler(void);
		void TIM3_IRQHandler(void);
		void TIM4_IRQHandler(void);
	public:
		F4Timer(TIM_TypeDef* TIMx);
		~F4Timer(){};
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(timer_callback cb);	
		void call_callback();
			
	};
}