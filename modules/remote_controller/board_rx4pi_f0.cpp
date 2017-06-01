#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0SysTimer.h>
#include <HAL/STM32F0/F0Timer.h>
#include <string.h>
#include "PPMOUT_f0.h"

using namespace STM32F0;
using namespace HAL;

HAL::ISPI *spi;
HAL::IGPIO *cs;
HAL::IGPIO *ce;
HAL::IGPIO *irq;
HAL::IGPIO *dbg;
HAL::IGPIO *dbg2;
HAL::IGPIO *SCL;
HAL::IGPIO *SDA;
HAL::IInterrupt *interrupt;
HAL::ITimer *timer;
int16_t adc_data[6] = {0};

namespace sheet1
{
	F0GPIO cs(GPIOB, GPIO_Pin_12);
	F0GPIO ce(GPIOB, GPIO_Pin_3);
	F0GPIO irq(GPIOA, GPIO_Pin_15);
	F0GPIO dbg(GPIOB, GPIO_Pin_11);
	
	F0GPIO dbg2(GPIOB, GPIO_Pin_10);
	F0GPIO SCL(GPIOC, GPIO_Pin_13);
	F0GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F0SPI spi;
	F0Interrupt interrupt;
	F0Timer timer(TIM14);
	
	
	int sheet1_init()
	{
		FLASH_SetLatency(FLASH_Latency_0);
		::cs = &cs;
		::ce = &ce;
		::irq = &irq;
		::dbg = &dbg;
		::dbg2 = &dbg2;
		::SCL = &SCL;
		::SDA = &SDA;
		::spi = &spi;
		::interrupt = &interrupt;
		::timer = &timer;
		
		spi.init(SPI2);
		interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
		
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		static PPMOUT ppmout;
		::ppm = &ppmout;		
		
		return 0;
	}	
}

int board_init()
{
	return sheet1::sheet1_init();
}



void read_channels(int16_t *channel, int max_channel_count)
{
}
