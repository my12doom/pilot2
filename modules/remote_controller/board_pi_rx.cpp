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
extern HAL::IUART *uart;
int16_t adc_data[6] = {0};
namespace sheet1
{
	F1GPIO cs(GPIOB, GPIO_Pin_12);
	F1GPIO ce(GPIOB, GPIO_Pin_3);
	F1GPIO irq(GPIOA, GPIO_Pin_15);
	F1GPIO dbg(GPIOB, GPIO_Pin_11);
	
	F1GPIO dbg2(GPIOB, GPIO_Pin_10);
	F1GPIO SCL(GPIOC, GPIO_Pin_13);
	F1GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F1SPI spi;
	F1Interrupt interrupt;
	F1Timer timer(TIM2);
	

	STM32F1::F1UART f1uart(USART1);	
	
	
	int sheet1_init()
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
		
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		
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
		//
		
		spi.init(SPI2);
		interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		
		
	
		static PPMOUT ppmout;
		::ppm = &ppmout;
		uart = &f1uart;
		
		systimer->delayms(20);
		while(0)
		{
			int16_t d[] = {1520, 1520, 1000, 1520,1100, 1200,0};
			ppmout.write(d, 4, 0);
			
			systimer->delayms(20);
		}
		
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



void read_channels(int16_t *channel, int max_channel_count)
{
}
