#include "stm32F4xx.h"
#include "F4GPIO.h"
#include "F4UART.h"
//#include "F4UART.h"
#include "stm32F4xx_gpio.h"
using namespace STM32F4;
using namespace HAL;
using namespace STM32F4;

F4UART * pUart4=new F4UART(UART4);

int main(void)
{
	
	uint8_t message[10];
	for(int k=0;k<9;k++)
		message[k]='K';
	F4GPIO P(GPIOC,GPIO_Pin_4);
	P.set_mode(MODE_OUT_PushPull);
	pUart4->set_baudrate(115200);
	pUart4->UART4_SendPacket("12345\n", 6);
	while(1)
	{
		P.write(0);
		for(int j=0;j<20;j++)
			for(int i=0;i<65535;i++);
		P.write(1);
		for(int j=0;j<20;j++)
			for(int i=0;i<65535;i++);
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

