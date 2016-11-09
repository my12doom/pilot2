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
F4GPIO dbg(GPIOC, GPIO_Pin_4);
F4GPIO dbg2(GPIOC, GPIO_Pin_5);
F4SPI spi1;
F4Interrupt interrupt;
F4Timer timer(TIM2);
uint8_t data[32];
randomizer<128, 512> rando;
uint16_t rand_pos = 0;
uint64_t seed = 0x1234567890345678;
OLED96 oled;
int o = 0;
int rdp;
int hoop_id = 0;
int miss = 99999;

extern "C" void TIM2_IRQHandler(void)
{
	timer.call_callback();
}

int hoop_to(int next_hoop_id)
{
	ce.write(false);
	
	if (next_hoop_id != hoop_id + 1)
		rando.reset(next_hoop_id);
	
	hoop_id = next_hoop_id;
	

	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
		
	nrf.write_reg(RF_CH, channel);
	
	ce.write(true);
	
	return 0;
}

int m = 0;

void nrf_irq_entry(void *parameter, int flags)
{
	int64_t t = systimer->gettime();
	dbg.write(false);
	timer.disable_cb();
	nrf.write_reg(7, nrf.read_reg(7));
	
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	int next_hoop_id = -1;
	while ((fifo_state&1) == 0)
	{
		nrf.read_rx(data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		o++;
		
		if (o % 5 == 0)
		next_hoop_id = *(uint16_t*)data;
	}
	
	if (next_hoop_id >= 0)
	{
		hoop_to(next_hoop_id+2);
		miss = 0;
		timer.restart();
	}
	
	timer.enable_cb();
	dbg.write(true);
	m = systimer->gettime() - t;
	
	if (m > 200)
		fputc('o', NULL);
}

void timer_entry()
{
	interrupt.disable();
	
	miss ++;
	
	if (miss < 1000)
	{
		dbg2.write(false);
		if (miss == 0)
			hoop_to((hoop_id+2)%65536);
		else
			hoop_to((hoop_id+1)%65536);
		dbg2.write(true);
	}
	
	interrupt.enable();
}

int main()
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);	
	
	spi1.init(SPI1);
	
	irq.set_mode(MODE_IN);
	dbg.set_mode(MODE_OUT_OpenDrain);
	dbg2.set_mode(MODE_OUT_OpenDrain);
	dbg.write(true);
	dbg2.write(true);
	
	rando.set_seed(seed);
	rand_pos = 0;
	while(nrf.init(&spi1, &cs, &ce) != 0)
		;
	nrf.write_reg(RF_CH, 95);
	nrf.rf_on(true);
	
	static F4GPIO SCL(GPIOC, GPIO_Pin_13);
	static F4GPIO SDA(GPIOC, GPIO_Pin_14);
	static I2C_SW i2c(&SCL, &SDA);
	oled.init(&i2c, 0x78);
	
	interrupt.init(GPIOA, GPIO_Pin_15, interrupt_falling);
	interrupt.set_callback(nrf_irq_entry, NULL);
		
	int64_t lt = systimer->gettime();
	
	int lo = 0;
	int64_t t = systimer->gettime();
	int lp = 0;
	
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	dbg.write(true);
	
	timer.set_callback(timer_entry);
	timer.set_period(2000);
	
	while(1)
	{		
		char tmp[100];
		sprintf(tmp, "%dp, pos=%d    ", o, *(uint16_t*)data);
		oled.show_str(0, 0, tmp);
		sprintf(tmp, "rdp=%d,%dp/s    ", rdp, lp);
		oled.show_str(0, 1, tmp);
		
		int64_t current = systimer->gettime();
		
		if (current - t > 1000000)
		{
			t = current;
			lp = o - lo;
			lo = o;
		}
	}
}
