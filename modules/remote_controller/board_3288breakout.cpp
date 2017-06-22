#include "board.h"

#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>

using namespace STM32F4;
using namespace HAL;


F4GPIO _cs(GPIOB, GPIO_Pin_5);
F4GPIO _ce(GPIOB, GPIO_Pin_4);
F4GPIO _irq(GPIOB, GPIO_Pin_1);
F4GPIO _dbg(GPIOB, GPIO_Pin_6);

F4GPIO _dbg2(GPIOC, GPIO_Pin_5);
F4GPIO _SCL(GPIOC, GPIO_Pin_13);
F4GPIO _SDA(GPIOC, GPIO_Pin_14);

F4SPI _spi;
F4Interrupt _interrupt;
F4Timer _timer(TIM2);

int board_init()
{		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	cs = &_cs;
	ce = &_ce;
	irq = &_irq;
	dbg = &_dbg;
	dbg2 = &_dbg2;
	SCL = &_SCL;
	SDA = &_SDA;
	spi = &_spi;
	interrupt = &_interrupt;
	timer = &_timer;		
	
	_spi.init(SPI1);		
	_interrupt.init(GPIOB, GPIO_Pin_1, interrupt_falling);
	
	return 0;
}

extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}


void read_channels(short *c, int count)
{
}
