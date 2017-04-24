#pragma once
#include <HAL/Interface/IGPIO.h>
#include <stm32f0xx_gpio.h>

namespace STM32F0
{
	class F0GPIO: public HAL::IGPIO
	{
	public:
		F0GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin);
	    ~F0GPIO(){};
		virtual void set_mode(HAL::GPIO_MODE mode);
		virtual bool read();					// high = true, low = false
		virtual void write(bool newvalue);		// high = true, low = false
		virtual void toggle();
	private:
		GPIO_TypeDef* GPIOx;
		uint32_t GPIO_Pin;
	};
}
