#pragma once

#include <stdint.h>
#include "IInterrupt.h"
#include "stm32f4xx.h"

namespace STM32F4
{
	class F4Interrupt:public HAL::IInterrupt
	{
	private:
		HAL::interrupt_callback cb;
		int flag;
		void *parameter;
	public:
		F4Interrupt();
		~F4Interrupt(){};
		bool init(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int flag);
		virtual void set_callback(HAL::interrupt_callback cb, void *parameter);
		virtual void call_callback();
	};
}