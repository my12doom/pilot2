#include "stm32F4xx.h"
#include "F4GPIO.h"
#include "F4UART.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
using namespace HAL;
using namespace STM32F4;
F4UART * pUart4=new F4UART(UART4);
int main(void)
{
	uint8_t message[10];
	for(int k=0;k<10;k++)
		message[k]=k;
	F4GPIO P(GPIOC,GPIO_Pin_4);
	P.set_mode(MODE_OUT_PushPull);
	pUart4->set_baudrate(115200);
	while(1)
	{
		pUart4->UART4_SendPacket(message,10);
		for(int j=0;j<5;j++)
			for(int i=0;i<65535;i++);
	}
}
void UART4_IRQHandler(void)
{
	pUart4->UART4_IRQHandler();
}
void DMA1_Stream4_IRQHandler()
{
	pUart4->DMA1_Steam4_IRQHandler();
}

