#pragma once

#include <stdint.h>
#include <HAL/Interface/IInterrupt.h>
#include "stm32f10x_gpio.h"

namespace STM32F1
{
	class F1Interrupt:public HAL::IInterrupt
	{
	private:
		HAL::interrupt_callback cb;
		int flag;
		void *parameter;
		GPIO_TypeDef* GPIOx;
		uint32_t GPIO_Pin;
	public:
		F1Interrupt();
		~F1Interrupt();
		bool init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int flag);
		void call_callback();
		virtual void set_callback(HAL::interrupt_callback cb, void *parameter);
		virtual int enable();
		virtual int disable();
	};
}