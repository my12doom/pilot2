#include <HAL/Interface/ISysTimer.h>
#include "board.h"
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
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
	
	rando.reset(0);
	hoop_id = 0;
	int c = nrf.init(spi, cs, ce);
	
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_callback(timer_entry);
	timer->set_period(2000);
	
	int64_t lt = systimer->gettime();
	
	while(1)
	{
	}
}
