#pragma once



#include "IGPIO.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"

namespace STM32F7_HAL
{
	typedef  uint32_t GPIOMode_TypeDef;
	class F7GPIO: public HAL::IGPIO
	{
	public:
		F7GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin);
	    ~F7GPIO(){};
		virtual void set_mode(HAL::GPIO_MODE mode);
		virtual bool read();					// high = true, low = false
		virtual void write(bool newvalue);		// high = true, low = false
		virtual void toggle();
	private:
		GPIO_TypeDef* GPIOx;
		uint32_t GPIO_Pin;  
		GPIOMode_TypeDef GPIO_Mode;
	};
}
