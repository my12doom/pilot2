#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/log.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <HAL/aux_devices/NRF24L01.h>

// BSP
using namespace STM32F4;
using namespace HAL;
using namespace devices;

NRF24L01 nrf;
F4SPI spi1;
void cb_entry(void *parameter, int flags)
{
//	cs.write(true);
	
}

int main()
{
	spi1.init(SPI1);
	F4GPIO cs(GPIOB, GPIO_Pin_5);
	F4GPIO ce(GPIOB, GPIO_Pin_4);
	F4GPIO irq(GPIOB, GPIO_Pin_1);
	
	F4Interrupt interrupt;
	interrupt.init(GPIOB, GPIO_Pin_1, interrupt_falling);
	interrupt.set_callback(cb_entry, NULL);
	
	int c = nrf.init(&spi1, &cs, &ce);
	
	uint8_t data[32];
	
	int64_t lt = systimer->gettime();
	
	while(1)
	{
		int fifo_state = nrf.read_reg(0x17);
		int state = nrf.read_reg(7);
		nrf.write_tx(data, 32);
		systimer->delayus(200);
		nrf.write_reg(7, nrf.read_reg(7));		

		if (systimer->gettime() - lt > 1000)
		{
			nrf.rf_off();
			nrf.write_reg(5, 90 + (rand()%100) * 5 / 100);
			systimer->delayus(100);
			nrf.rf_on(false);
			lt = systimer->gettime();
		}
	}
}
