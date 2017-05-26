#include <HAL/Interface/IStorage.h>
#include <HAL/Interface/ISysTimer.h>
#include <HAL/STM32F0/F0GPIO.h>
#include <HAL/STM32F0/F0SPI.h>
#include <stdlib.h>

using namespace STM32F0;
using namespace HAL;

HAL::IStorage *get_default_storage()
{
	return NULL;
}

int main()
{
	F0GPIO led(GPIOB, GPIO_Pin_11);
	led.set_mode(MODE_OUT_OpenDrain);
	F0SPI spi(SPI1);
	led.write(true);
	
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_ClockSecuritySystemCmd(ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_MCOConfig(RCC_MCOSource_SYSCLK);
	
	spi.set_speed(25000000);
	spi.set_mode(0,0);
	uint8_t tx[30];
	for(int i=0; i<30; i++)
		tx[i] = i;
		
	uint8_t rx[30];
	spi.txrx2(tx, rx, 30);
	led.toggle();
	systimer->delayus(16);
	
	while(1)
	{
		led.write(false);
		systimer->delayus(150);
		led.write(true);
		systimer->delayus(150);
	}
}
