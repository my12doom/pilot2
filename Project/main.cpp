#include "stm32F4xx.h"
#include "STM32F4GPIO.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
using namespace HAL;
int main(void)
{
	
	F4GPIO P(GPIOC,GPIO_Pin_4);
	P.set_mode(MODE_OUT_PushPull);
	while(1)
	{
		P.write(1);
		for(__IO int j=0;j<20;j++)
		for(__IO int i=0;i<65535;i++);
		P.write(0);
		for(__IO int k=0;k<20;k++)
		for(__IO int n=0;n<65535;n++);
		
	}
}
