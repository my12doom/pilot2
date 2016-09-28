#include "F1GPIO.h"
#include "stm32f10x_rcc.h"
#include <stdint.h>

using namespace HAL;

namespace STM32F1
{
	F1GPIO::F1GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE,ENABLE);
		this->GPIOx = GPIOx;
		this->GPIO_Pin    = GPIO_Pin;
	}
	void F1GPIO::set_mode(GPIO_MODE mode)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin  = GPIO_Pin;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		if(MODE_IN == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
		}
		else if(MODE_OUT_PushPull == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		}
		else if(MODE_OUT_OpenDrain == mode)
		{
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
		}	
		GPIO_Init(this->GPIOx, &GPIO_InitStructure);
	}
	bool F1GPIO::read()
	{
		volatile int v;
		//GPIOx->MODER|=GPIO_Mode_IN<<26;
		v = GPIOx->IDR & GPIO_Pin;
		//GPIOx->MODER|=GPIO_Mode_OUT<<26;
		return v;

	}
	void F1GPIO::write(bool newvalue){
		if(newvalue)
		{
			GPIO_SetBits(this->GPIOx,this->GPIO_Pin);
		}
		else
		{
			GPIO_ResetBits(this->GPIOx,this->GPIO_Pin);
		}	
	}
	void F1GPIO::toggle()
	{
		GPIOx->ODR ^= GPIO_Pin;
	}
}
