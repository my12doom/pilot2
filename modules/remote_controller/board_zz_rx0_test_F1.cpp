#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/STM32F1/F1VCP.h>
#include <HAL/Interface/II2C.h>
#include <HAL/Interface/ISysTimer.h>
#include <string.h>
#include <utils/space.h>
#include <misc.h>

using namespace STM32F1;
using namespace HAL;


F1GPIO _cs(GPIOA, GPIO_Pin_8);
F1GPIO _ce(GPIOA, GPIO_Pin_9);
F1GPIO _irq(GPIOA, GPIO_Pin_10);

F1GPIO _dbg(GPIOA, GPIO_Pin_4);
F1GPIO _dbg2(GPIOA, GPIO_Pin_5);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);
F1VCP vcp;

extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}

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
	::telemetry = &vcp;
	
	while(0)
	{
		vcp.write("hello\n", 6);
		systimer->delayms(100);
		
		while(vcp.available() > 0)
		{
			uint8_t tmp[20];
			int n = vcp.read(tmp, sizeof(tmp));
			vcp.write(tmp, n);
		}
	}
	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_10, interrupt_falling);
			
	_dbg.set_mode(HAL::MODE_OUT_OpenDrain);
	_dbg2.set_mode(HAL::MODE_OUT_OpenDrain);
			
	_dbg.write(0);
	systimer->delayms(100);
	return 0;
}

void read_channels(int16_t *channel, int max_channel_count)
{
	for(int i=0; i<max_channel_count; i++)
		channel[i] = i*3;
}
