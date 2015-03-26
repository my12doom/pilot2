#include "stm32F4xx.h"
#include "F4GPIO.h"
#include "F4UART.h"
#include "F4SPI.h"
#include "F4Timer.h"
#include "F4SysTimer.h"
#include "BSP/devices/sensors/MS5611_SPI.h"
#include <BSP/boards/dev_v1/RCIN.h>
#include <BSP/boards/dev_v1/RCOUT.h>
#include <stdio.h>

//#include "F4UART.h"
#include "stm32F4xx_gpio.h"

using namespace STM32F4;
using namespace HAL;
using namespace sensors;

F4UART * pUart4=new F4UART(UART4);
uint8_t recv_buffer[5];
dev_v1::RCIN rc;
dev_v1::RCOUT rcout;

void delay()
{
	int64_t t = systimer->gettime();
	while(systimer->gettime() - t < 20)
		;
}

extern "C" int64_t getus()
{
	return systimer->gettime();
}

extern "C" void delayms(int ms)
{
	systimer->delayms(ms);
}
Timer * pTIM1=new F4Timer(TIM1);
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);

	F4GPIO cs(GPIOE,GPIO_Pin_7);
	cs.set_mode(MODE_OUT_PushPull);
	F4SPI spi2(SPI2);
	MS5611_SPI baro(&spi2, &cs);
	baro.init();
	//Test Timer:
	while(1)
	{
		
	}
	while(1)
	{		
		int16_t rcs[8];
		rc.get_channel_data(rcs, 0, 8);

		rcout.write(rcs, 0, 6);
		systimer->delayms(10);
		
		//printf("\r%d,%d,%d,%d,%d,%d", rcs[0], rcs[1], rcs[2], rcs[3], rcs[4], rcs[5]);
	}
	
	while(1)
	{
		int data[2];
		int res = baro.read(data);
		//int res = read_MS5611spi(data);
		printf("\r%d, %d, %d", res, data[0], data[1]);
		
 		res = res;
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


struct __FILE { int handle; /* Add whatever you need here */ };
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

extern "C" int fputc(int ch, FILE *f)
{
	if (DEMCR & TRCENA) 
	{
		while (ITM_Port32(0) == 0);
		ITM_Port8(0) = ch;
	}
	return (ch);
}

