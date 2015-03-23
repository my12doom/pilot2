#include "stm32F4xx.h"
#include "STM32F4GPIO.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
int main(void)
{
	
	while(1)
	{
		F4GPIO P(GPIOC,GPIO_Pin_4,GPIO_Mode_OUT);
		P.write(1);
	}
}
