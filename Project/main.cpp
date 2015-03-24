#include "stm32F4xx.h"
#include "F4GPIO.h"
#include "F4UART.h"
#include "F4SysTimer.h"

//#include "F4UART.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
using namespace HAL;
using namespace STM32F4;

F4UART * pUart4=new F4UART(UART4);
uint8_t recv_buffer[5];

void delay()
{
	int64_t t = systimer->gettime();
	while(systimer->gettime() - t < 20)
		;
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	F4GPIO P(GPIOA,GPIO_Pin_7);
	P.set_mode(MODE_OUT_PushPull);
	
	while(1)
	{
		P.write(true);
		delay();
		P.write(false);
		delay();
	}
}

extern "C" void UART4_IRQHandler(void)
{
	pUart4->UART4_IRQHandler();
}
extern "C" void DMA1_Stream4_IRQHandler()
{
	pUart4->DMA1_Steam4_IRQHandler();
}

