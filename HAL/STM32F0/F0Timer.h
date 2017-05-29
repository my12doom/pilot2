#pragma once

#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include <stm32F0xx_tim.h>

namespace STM32F0
{
	class F0Timer:public HAL::ITimer
	{
	private:
		HAL::timer_callback cb;
		TIM_TypeDef* TIMx;
		IRQn_Type IRQn;
		void *user_data;
		void TimerInit(TIM_TypeDef* TIMx);
	public:
		F0Timer(TIM_TypeDef* TIMx);
		~F0Timer(){};
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(HAL::timer_callback cb, void *user_data);	
		virtual void call_callback();
		virtual void restart();
		virtual void enable_cb();
		virtual void disable_cb();		
	};
}