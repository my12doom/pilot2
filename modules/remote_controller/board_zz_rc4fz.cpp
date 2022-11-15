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

#include "PPMOUT_f1.h"

using namespace STM32F1;
using namespace HAL;

int16_t adc_data[6] = {0};

F1GPIO _cs(GPIOA, GPIO_Pin_8);
F1GPIO _ce(GPIOA, GPIO_Pin_9);
F1GPIO _irq(GPIOA, GPIO_Pin_10);

F1GPIO _dbg(GPIOC, GPIO_Pin_13);
F1GPIO _dbg2(GPIOC, GPIO_Pin_14);
F1GPIO power_sta(GPIOA, GPIO_Pin_11);
F1GPIO power_on(GPIOA, GPIO_Pin_12 | GPIO_Pin_7);

F1SPI _spi;
F1Interrupt _interrupt;
F1Timer _timer(TIM2);
F1Timer button_timer(TIM4);

F1GPIO ants[2] =
{
	F1GPIO(GPIOB, GPIO_Pin_0),
	F1GPIO(GPIOB, GPIO_Pin_1),
};

//#define DSM

STM32F1::F1UART _sbus(USART2);
STM32F1::F1UART tele(USART3);

	
extern "C" void TIM2_IRQHandler(void)
{
	_timer.call_callback();
}
extern "C" void TIM4_IRQHandler(void)
{
	button_timer.call_callback();
}

const int off_timeout = 300;
int off_timer = off_timeout;
void button_timer_entry(void *p)
{
	if (!power_sta.read())
		off_timer++;
	else
		off_timer = 0;
	
	if (off_timer == off_timeout)
		power_on.write(0);
	
	if (off_timer > off_timeout)
		off_timer = off_timeout;
}

int board_init()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
	bool skip_boot = RCC->CSR & 0xE0000000;
	power_on.set_mode(HAL::MODE_OUT_PushPull);
	power_on.write(0);
	power_on.write(skip_boot);
	
	cs = &_cs;
	ce = &_ce;
	irq = &_irq;
	dbg = &_dbg;
	dbg2 = &_dbg2;
	SCL = NULL;
	SDA = NULL;
	spi = &_spi;
	interrupt = &_interrupt;
	timer = &_timer;
	
	sbus = &_sbus;
	telemetry = &tele;	
	tele.set_baudrate(500000);


	
	//	
	_spi.init(SPI2);
	_interrupt.init(GPIOA, GPIO_Pin_10, interrupt_falling);
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	

	static PPMOUT ppmout;
	ppm = &ppmout;

	power_sta.set_mode(HAL::MODE_IN);

	int counter = 0;
	while (counter < 50 && !skip_boot)
	{
		systimer->delayms(10);
		counter++;
		if (power_sta.read())
			counter = 0;
	}
	
	power_on.write(1);

	button_timer.set_period(10000);
	button_timer.set_callback(button_timer_entry, NULL);
	button_timer.set_priority(4);
	
	return 0;
}

void select_ant(uint32_t *randomizer, bool tx)
{
	for(int i=0; i<2; i++)
	{
		ants[i].set_mode(MODE_OUT_PushPull);
		ants[i].write(!((randomizer[1]+i)&1));
	}
}
