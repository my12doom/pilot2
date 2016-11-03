#include <HAL/STM32F4/F4SPI.h>
#include <HAL/STM32F4/F4GPIO.h>
#include <HAL/STM32F4/F4Interrupt.h>
#include <HAL/STM32F4/F4Timer.h>
#include <HAL/Interface/ISysTimer.h>
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <HAL/aux_devices/NRF24L01.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include "randomizer.h"

// BSP
using namespace STM32F4;
using namespace HAL;
using namespace devices;

NRF24L01 nrf;
F4GPIO cs(GPIOC, GPIO_Pin_3);
F4GPIO ce(GPIOC, GPIO_Pin_2);
F4GPIO irq(GPIOA, GPIO_Pin_15);
F4GPIO dbg(GPIOC, GPIO_Pin_6);
F4SPI spi1;
F4Interrupt interrupt;
F4Timer timer(TIM2);
uint8_t data[32];
randomizer rando;
uint16_t rand_pos = 0;
uint64_t seed = 0x1234567890345678;
OLED96 oled;
int o = 0;
int rdp;

extern "C" void TIM2_IRQHandler(void)
{
	timer.call_callback();
}

void nrf_irq_entry(void *parameter, int flags)
{
	nrf.write_reg(7, nrf.read_reg(7));
	o++;
	
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	while ((fifo_state&1) == 0)
	{
		nrf.read_rx(data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		o++;
	}
}

void timer_entry()
{
	
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
	nrf.write_reg(RF_CH, 95);
	nrf.rf_on(true);
	
	static F4GPIO SCL(GPIOC, GPIO_Pin_13);
	static F4GPIO SDA(GPIOC, GPIO_Pin_14);
	static I2C_SW i2c(&SCL, &SDA);
	oled.init(&i2c, 0x78);
	
	interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
	interrupt.set_callback(nrf_irq_entry, NULL);
	timer.set_callback(timer_entry);
	timer.set_period(1500);
		
	int64_t lt = systimer->gettime();
	
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	while(1)
	{
		interrupt.disable();
			
		int state = nrf.read_reg(STATUS);
		int fifo_state = nrf.read_reg(FIFO_STATUS);
				
		interrupt.enable();
		
		char tmp[100];
		sprintf(tmp, "%dp, pos=%d    ", o, *(int16_t*)data);
		oled.show_str(0, 0, tmp);		
		sprintf(tmp, "rdp=%d    ", rdp);
		oled.show_str(0, 1, tmp);		
		
		systimer->delayms(50);
	}
}
