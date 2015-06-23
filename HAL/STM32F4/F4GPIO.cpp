#include "F4GPIO.h"
#include "stm32f4xx_rcc.h"
#include <stdint.h>

using namespace HAL;

namespace STM32F4
{
	F4GPIO::F4GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin)
	{
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_GPIOE,ENABLE);
		this->GPIOx = GPIOx;
		this->GPIO_Pin    = GPIO_Pin;
	}
	void F4GPIO::set_mode(GPIO_MODE mode)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin  = GPIO_Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
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
			GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		}	
		GPIO_Init(this->GPIOx, &GPIO_InitStructure);
	}
	bool F4GPIO::read()
	{
		volatile int v;
		//GPIOx->MODER|=GPIO_Mode_IN<<26;
		v = GPIOx->IDR & GPIO_Pin;
		//GPIOx->MODER|=GPIO_Mode_OUT<<26;
		return v;

	}
	void F4GPIO::write(bool newvalue){
		if(newvalue)
		{
			GPIO_SetBits(this->GPIOx,this->GPIO_Pin);
		}
		else
		{
			GPIO_ResetBits(this->GPIOx,this->GPIO_Pin);
		}	
	}
	void F4GPIO::toggle()
	{
		GPIO_ToggleBits(GPIOx, GPIO_Pin);
	}
}
