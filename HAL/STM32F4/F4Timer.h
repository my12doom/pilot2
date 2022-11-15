#pragma once

#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include "stm32f4xx.h"

namespace STM32F4
{
	class F4Timer:public HAL::ITimer
	{
	private:
		HAL::timer_callback cb;
		TIM_TypeDef* TIMx;
		IRQn_Type IRQn;
		void TimerInit(TIM_TypeDef* TIMx);
	public:
		F4Timer(TIM_TypeDef* TIMx);
		~F4Timer(){};
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(HAL::timer_callback cb, void *user_data = 0);	
		virtual void call_callback();
		virtual void restart();
		virtual void enable_cb();
		virtual void disable_cb();
		virtual void set_priority(int preemption_priority, int sub_priority = 0);
			
		void *user_data;
	};
}
