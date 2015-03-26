#pragma once

#include <stdint.h>
#include "Timer.h"
#include "stm32f4xx.h"
using namespace HAL;
namespace STM32F4
{
	class F4Timer:public Timer
	{
	private:
		TIM_TypeDef* TIMx;
		void TimerInit(TIM_TypeDef* TIMx);
	public:
		F4Timer(TIM_TypeDef* TIMx);
		~F4Timer(){};
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(timer_callback cb);	
			
	};
}