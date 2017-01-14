#pragma once

#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include <stm32f10x_tim.h>

namespace STM32F1
{
	class F1Timer:public HAL::ITimer
	{
	private:
		HAL::timer_callback cb;
		TIM_TypeDef* TIMx;
		IRQn_Type IRQn;
		void *user_data;
		void TimerInit(TIM_TypeDef* TIMx);
	public:
		F1Timer(TIM_TypeDef* TIMx);
		~F1Timer(){};
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(HAL::timer_callback cb, void *user_data);	
		virtual void call_callback();
		virtual void restart();
		virtual void enable_cb();
		virtual void disable_cb();		
	};
}