#include "stm32F4xx.h"
#include "F4GPIO.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
using namespace HAL;
int main(void)
{
	
	F4GPIO P(GPIOC,GPIO_Pin_4);
	P.set_mode(MODE_OUT_PushPull);
	while(1)
	{
		
	}
}
