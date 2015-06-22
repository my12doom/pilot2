#pragma once

#include <stdint.h>
#include "ITimer.h"
#include "stm32f4xx.h"

namespace STM32F4
{
	class F4Timer:public HAL::ITimer
	{
	private:
		HAL::timer_callback cb;
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
		virtual void set_callback(HAL::timer_callback cb);	
		virtual void call_callback();
			
	};
}