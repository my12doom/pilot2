#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <HAL/aux_devices/NRF24L01.h>
#include "randomizer.h"

// BSP
using namespace STM32F4;
using namespace HAL;
using namespace devices;

NRF24L01 nrf;
F4GPIO cs(GPIOB, GPIO_Pin_5);
F4GPIO ce(GPIOB, GPIO_Pin_4);
F4GPIO irq(GPIOB, GPIO_Pin_1);
F4GPIO dbg(GPIOB, GPIO_Pin_6);	
F4SPI spi1;
F4Interrupt interrupt;
F4Timer timer(TIM2);
uint8_t data[32];
randomizer rando;
uint16_t rand_pos = 0;
uint64_t seed = 0x1234567890345678;

extern "C" void TIM2_IRQHandler(void)
{
	timer.call_callback();
}

void nrf_irq_entry(void *parameter, int flags)
{
	nrf.write_reg(7, nrf.read_reg(7));
}

void timer_entry()
{
	interrupt.disable();
	
	*(uint16_t*)data = rand_pos;
	rand_pos ++;
	
	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
	
	if (rand_pos == 0)
		rando.reset(seed);
	
	nrf.rf_off();
	nrf.write_reg(5, channel);
	nrf.write_tx(data, 32);
	nrf.rf_on(false);	
	
	interrupt.enable();
	
	dbg.write(true);
	systimer->delayus(1);
	dbg.write(false);
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	spi1.init(SPI1);
	
	irq.set_mode(MODE_IN);
	dbg.set_mode(MODE_OUT_PushPull);
	dbg.write(false);	
	
	rando.reset(seed);
	rand_pos = 0;
	int c = nrf.init(&spi1, &cs, &ce);
	
	interrupt.init(GPIOB, GPIO_Pin_1, interrupt_falling);
	interrupt.set_callback(nrf_irq_entry, NULL);
	timer.set_callback(timer_entry);
	timer.set_period(1500);
	
	int64_t lt = systimer->gettime();
	
	while(1)
	{
	}
}
