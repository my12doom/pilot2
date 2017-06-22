#include "board.h"

#include <HAL/STM32F1/F1SPI.h>
#include <HAL/STM32F1/F1GPIO.h>
#include <HAL/STM32F1/F1Interrupt.h>
#include <HAL/STM32F1/F1SysTimer.h>
#include <misc.h>
#include <HAL/STM32F1/F1Timer.h>
#include <HAL/STM32F1/F1UART.h>
#include <stm32f10x.h>
#include <string.h>

#include "BrushedOut.h"

using namespace STM32F1;
using namespace HAL;

int16_t adc_data[6] = {0};


F1GPIO _cs(GPIOB, GPIO_Pin_12);
F1GPIO _ce(GPIOB, GPIO_Pin_3);
F1GPIO _irq(GPIOA, GPIO_Pin_15);
F1GPIO _dbg(GPIOB, GPIO_Pin_11);

F1GPIO _dbg2(GPIOB, GPIO_Pin_10);
F1GPIO _SCL(GPIOC, GPIO_Pin_13);
F1GPIO _SDA(GPIOC, GPIO_Pin_14);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);


STM32F1::F1UART f1uart(USART1);	


int board_init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		
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
	//
	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
			

	static BrushedOut ppmout;
	::ppm = &ppmout;
	uart = &f1uart;
	
	return 0;
}


extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}

void read_channels(int16_t *channel, int max_channel_count)
{
}
