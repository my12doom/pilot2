#include "board.h"

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>

using namespace STM32F4;
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

namespace sheet1
{
	F4GPIO cs(GPIOB, GPIO_Pin_5);
	F4GPIO ce(GPIOB, GPIO_Pin_4);
	F4GPIO irq(GPIOB, GPIO_Pin_1);
	F4GPIO dbg(GPIOB, GPIO_Pin_6);
	
	F4GPIO dbg2(GPIOC, GPIO_Pin_5);
	F4GPIO SCL(GPIOC, GPIO_Pin_13);
	F4GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F4SPI spi;
	F4Interrupt interrupt;
	F4Timer timer(TIM2);
	
	int sheet1_init()
	{		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
		
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
		
		spi.init(SPI1);		
		interrupt.init(GPIOB, GPIO_Pin_1, interrupt_falling);
		
		return 0;
	}
	
	extern "C" void TIM2_IRQHandler(void)
	{
		timer.call_callback();
	}
}

using namespace sheet1;

int board_init()
{
	return sheet1_init();
}
