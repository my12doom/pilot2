#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0SysTimer.h>
#include <HAL/STM32F0/F0Timer.h>
#include <string.h>
#include "PPMOUT_f0.h"
#include <HAL/STM32F0/F0UART.h>

using namespace STM32F0;
using namespace HAL;


F0GPIO _cs(GPIOB, GPIO_Pin_12);
F0GPIO _ce(GPIOB, GPIO_Pin_3);
F0GPIO _irq(GPIOA, GPIO_Pin_15);
F0GPIO _dbg(GPIOB, GPIO_Pin_11);
F0GPIO _dbg2(GPIOB, GPIO_Pin_10);

F0SPI _spi;
F0Interrupt _interrupt;
F0Timer _timer(TIM14);
F0UART uart(USART1);

int board_init()
{
	::cs = &_cs;
	::ce = &_ce;
	::irq = &_irq;
	::dbg = &_dbg;
	::dbg2 = &_dbg2;
	::spi = &_spi;
	::interrupt = &_interrupt;
	::timer = &_timer;
	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI4_15_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	static PPMOUT ppmout;
	::ppm = &ppmout;
	//::telemetry = &uart;
			
	return 0;
}

void read_channels(int16_t *channel, int max_channel_count)
{
}
