#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <misc.h>
#include <HAL/STM32F1/F1Timer.h>

using namespace STM32F1;
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
	F1GPIO cs(GPIOB, GPIO_Pin_5);
	F1GPIO ce(GPIOB, GPIO_Pin_4);
	F1GPIO irq(GPIOB, GPIO_Pin_1);
	F1GPIO dbg(GPIOB, GPIO_Pin_6);
	
	F1GPIO dbg2(GPIOC, GPIO_Pin_5);
	F1GPIO SCL(GPIOC, GPIO_Pin_13);
	F1GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F1SPI spi;
	F1Interrupt interrupt;
	F1Timer timer(TIM2);
	
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
