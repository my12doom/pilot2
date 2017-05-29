#pragma once

#include <stdint.h>
#include <HAL/Interface/IInterrupt.h>
#include "stm32F0xx_gpio.h"

namespace STM32F0
{
	class F0Interrupt:public HAL::IInterrupt
	{
	private:
		HAL::interrupt_callback cb;
		int flag;
		void *parameter;
		GPIO_TypeDef* GPIOx;
		uint32_t GPIO_Pin;
	public:
		F0Interrupt();
		~F0Interrupt();
		bool init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int flag);
		void call_callback();
		virtual void set_callback(HAL::interrupt_callback cb, void *parameter);
		virtual int enable();
		virtual int disable();
	};
}