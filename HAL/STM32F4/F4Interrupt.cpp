#include "F4Interrupt.h"
using namespace HAL;
namespace STM32F4
{
	F4Interrupt::F4Interrupt(GPIO_TypeDef* GPIOx, uint32_t GPIO_Pin, int flag)
	{
	}
	void F4Interrupt::set_callback(HAL::interrupt_callback cb, void *parameter)
	{		
		this->cb=cb;
		this->parameter = parameter;
	}
	void F4Interrupt::call_callback()
	{
		if(cb)
			cb(parameter, flag);
	}
}
