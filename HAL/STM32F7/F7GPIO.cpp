#include "F7GPIO.h"
//#include "stm32f7xx_hal_gpio.h"
#include <stdint.h>

using namespace HAL;

namespace STM32F7_HAL
{
	F7GPIO::F7GPIO(GPIO_TypeDef* GPIOx,uint32_t GPIO_Pin)
	{
		__GPIOE_CLK_ENABLE();
		__GPIOG_CLK_ENABLE();
		__GPIOB_CLK_ENABLE();
		__GPIOD_CLK_ENABLE();
		__GPIOC_CLK_ENABLE();
		__GPIOA_CLK_ENABLE();
		__GPIOJ_CLK_ENABLE();
		__GPIOI_CLK_ENABLE();
		__GPIOK_CLK_ENABLE();
		__GPIOF_CLK_ENABLE();
		__GPIOH_CLK_ENABLE();
		this->GPIOx = GPIOx;
		this->GPIO_Pin    = GPIO_Pin;
	}
	void F7GPIO::set_mode(GPIO_MODE mode)
	{
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.Pin  = GPIO_Pin;
		GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
		if(MODE_IN == mode)
		{
			GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
			GPIO_InitStructure.Pull = GPIO_PULLUP;
		}
		else if(MODE_OUT_PushPull == mode)
		{
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP  ;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
		}
		else if(MODE_OUT_OpenDrain == mode)
		{
			GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD ;
			GPIO_InitStructure.Pull = GPIO_NOPULL;
		}	
		HAL_GPIO_Init(this->GPIOx, &GPIO_InitStructure);
	}
	bool F7GPIO::read()
	{
		volatile int v;
		v=HAL_GPIO_ReadPin(this->GPIOx,this->GPIO_Pin);
		return v;

	}
	void F7GPIO::write(bool newvalue){
		if(newvalue)
		{
			HAL_GPIO_WritePin(this->GPIOx,this->GPIO_Pin, GPIO_PIN_SET);
			
		}
		{
			HAL_GPIO_WritePin(this->GPIOx,this->GPIO_Pin, GPIO_PIN_RESET);
		}	
	}
	void F7GPIO::toggle()
	{
		HAL_GPIO_TogglePin(this->GPIOx,this->GPIO_Pin);
	}
}
