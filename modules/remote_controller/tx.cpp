#include <HAL/Interface/ISysTimer.h>
#include "board.h"
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <HAL/aux_devices/NRF24L01.h>
#include "randomizer.h"

// BSP
using namespace HAL;
using namespace devices;

NRF24L01 nrf;
uint8_t data[32];
randomizer<256,256> rando;
uint16_t hoop_id = 0;
uint64_t seed = 0x1234567890345678;
uint16_t hoop_interval = 1000;

int64_t ts;
int dt;

void nrf_irq_entry(void *parameter, int flags)
{
	dt = systimer->gettime() - ts;
	nrf.write_reg(7, nrf.read_reg(7));				// sending 32bytes payload with 3byte address and 2byte CRC cost ~1373us
													// delay between tx and rx is 25us max
}

void timer_entry()
{
	interrupt->disable();
	dbg->write(true);
	
	*(uint16_t*)data = hoop_id;
	hoop_id ++;
	
	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
	data[2] = channel;
	
	//channel = 85;
	
	read_channels((int16_t*)(data+2), 6);
	
	if (hoop_id == 0)
		rando.reset();
	
	ce->write(false);
	nrf.write_reg(5, channel);
	nrf.write_tx(data, 32);
	ce->write(true);
	interrupt->enable();
	ts = systimer->gettime();
	dbg->write(false);
}

int main()
{
	board_init();
	
	irq->set_mode(MODE_IN);
	dbg->set_mode(MODE_OUT_PushPull);
	dbg->write(false);
	dbg2->set_mode(MODE_OUT_PushPull);
	dbg2->write(true);
	
	rando.reset(0);
	hoop_id = 0;
	int c = 1;
	while( (c = nrf.init(spi, cs, ce)) != 0)
	{
		dbg2->write(false);
		systimer->delayms(100);
		dbg2->write(true);
		systimer->delayms(100);
	}
	dbg2->write(false);
		
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
	
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_callback(timer_entry);
	timer->set_period(hoop_interval);
	
	int64_t lt = systimer->gettime();
	
	int16_t v[6] = {0};
	while(1)
	{
	}
}
