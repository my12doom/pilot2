#include "board.h"

#include <HAL/STM32F0/F0SPI.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0Interrupt.h>
#include <HAL/STM32F0/F0Timer.h>
#include <string.h>

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
	F0GPIO cs(GPIOC, GPIO_Pin_6);
	F0GPIO ce(GPIOC, GPIO_Pin_7);
	F0GPIO irq(GPIOB, GPIO_Pin_12);
	F0GPIO dbg2(GPIOC, GPIO_Pin_9);
	
	F0GPIO dbg(GPIOC, GPIO_Pin_4);
	F0GPIO SCL(GPIOC, GPIO_Pin_13);
	F0GPIO SDA(GPIOC, GPIO_Pin_14);
	
	F0SPI spi;
	F0Interrupt interrupt;
	F0Timer timer(TIM14);
	
	F0GPIO pa6(GPIOA, GPIO_Pin_6);
	F0GPIO pc8(GPIOC, GPIO_Pin_8);

	
	
	int sheet1_init()
	{		
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
		::bind_button = &pc8;
		pc8.set_mode(MODE_IN);
		
		spi.init(SPI2);
		interrupt.init(GPIOB, GPIO_Pin_12, interrupt_falling);
		
		pa6.set_mode(MODE_IN);
				
		return 0;
	}
}

using namespace sheet1;

int board_init()
{
	return sheet1_init();
}



void read_channels(int16_t *channel, int max_channel_count)
{
	if (max_channel_count > sizeof(adc_data)/2)
		max_channel_count = sizeof(adc_data)/2;
	
	memcpy(channel, adc_data, max_channel_count * 2);	
}
