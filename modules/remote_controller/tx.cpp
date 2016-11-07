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
randomizer<256,256> rando;
uint16_t hoop_id = 0;
uint64_t seed = 0x1234567890345678;

int64_t ts;
int dt;

extern "C" void TIM2_IRQHandler(void)
{
	timer.call_callback();
}

void nrf_irq_entry(void *parameter, int flags)
{
	dt = systimer->gettime() - ts;
	nrf.write_reg(7, nrf.read_reg(7));				// sending 32bytes payload with 3byte address and 2byte CRC cost ~1373us
													// delay between tx and rx is 25us max
}

void timer_entry()
{
	interrupt.disable();
	dbg.write(true);
	
	*(uint16_t*)data = hoop_id;
	hoop_id ++;
	
	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
	data[2] = channel;
	
	if (hoop_id == 0)
		rando.reset();
	
	ce.write(false);
	nrf.write_reg(5, channel);
	nrf.write_tx(data, 32);
	ce.write(true);
	interrupt.enable();
	ts = systimer->gettime();
	dbg.write(false);
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
	spi1.init(SPI1);
	
	irq.set_mode(MODE_IN);
	dbg.set_mode(MODE_OUT_PushPull);
	dbg.write(false);	
	
	rando.reset(0);
	hoop_id = 0;
	int c = nrf.init(&spi1, &cs, &ce);
	
	interrupt.init(GPIOB, GPIO_Pin_1, interrupt_falling);
	interrupt.set_callback(nrf_irq_entry, NULL);
	timer.set_callback(timer_entry);
	timer.set_period(2000);
	
	int64_t lt = systimer->gettime();
	
	while(1)
	{
	}
}
