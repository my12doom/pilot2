#include "F0GPIO.h"
#include <stm32f0xx_rcc.h>
#include <stdint.h>

using namespace HAL;

namespace STM32F0
{
	F0GPIO::F0GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin)
	{
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
		this->GPIOx = GPIOx;
		this->GPIO_Pin    = GPIO_Pin;
	}
	void F0GPIO::set_mode(GPIO_MODE mode)
	{
		GPIO_InitTypeDef GPIO_InitStructure = {0};
		GPIO_InitStructure.GPIO_Pin  = GPIO_Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
		if(MODE_IN == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		}
		else if(MODE_OUT_PushPull == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		}
		else if(MODE_OUT_OpenDrain == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		}
		GPIO_Init(this->GPIOx, &GPIO_InitStructure);
	}
	
	bool F0GPIO::read()
	{
		return GPIOx->IDR & GPIO_Pin;
	}
	void F0GPIO::write(bool newvalue)
	{
		if(newvalue)
			GPIOx->BSRR = GPIO_Pin;
		else
			GPIOx->BRR = GPIO_Pin;
	}
	void F0GPIO::toggle()
	{
		GPIOx->ODR ^= GPIO_Pin;
	}
}
