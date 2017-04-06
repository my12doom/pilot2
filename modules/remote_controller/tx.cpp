#include <HAL/Interface/ISysTimer.h>
#include "board.h"
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <HAL/aux_devices/NRF24L01.h>
#include <utils/space.h>
#include <utils/RIJNDAEL.h>
#include "randomizer.h"
#include "binding.h"
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
AESCryptor aes;
HAL::IGPIO *bind_button = NULL;

void nrf_irq_entry(void *parameter, int flags)
{
	dt = systimer->gettime() - ts;
	nrf.write_reg(7, nrf.read_reg(7));				// sending 32bytes payload with 3byte address and 2byte CRC cost ~1373us
													// delay between tx and rx is 25us max
}

void timer_entry(void *p)
{
	interrupt->disable();
	dbg->write(true);
	
	*(uint16_t*)data = hoop_id;
	hoop_id ++;
	
	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
	data[2] = channel;
	
	//channel = 85;
	
	read_channels((int16_t*)(data+2), 6);
	
	*(uint16_t*)(data+30) = crc32(0, data, 30);
	
	aes.encrypt(data, data);
	aes.encrypt(data+16, data+16);
	
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

int binding_loop()
{	
	nrf.set_rx_address(0, (uint8_t*)&seed, 3);
	nrf.set_rx_address(1, (uint8_t*)BINDING_ADDRESS, 3);
	nrf.enable_rx_address(0, true);
	nrf.enable_rx_address(1, true);
	nrf.set_tx_address(BINDING_ADDRESS, 3);
	nrf.write_reg(RF_CH, BINDING_CHANNEL);
	nrf.rf_on(false);
	
	// binding packets
	binding_pkt pkt = {{'B','D'}, 0, 0};
	binding_info_v0 *payload = (binding_info_v0 *)pkt.payload;
	payload->cmd = cmd_requesting_binding;
	memcpy(payload->key, &seed, 8);
	pkt.crc = crc32(0, ((uint8_t*)&pkt)+3, 29);
	
	int64_t t = systimer->gettime() + 5000000000;
	
	while(systimer->gettime() < t)
	{
		// send all packets out
		nrf.rf_off();
		nrf.rf_on(false);
		nrf.write_tx((uint8_t*)&pkt, 32);
		systimer->delayms(3);
		
		// wait for ack packet
		nrf.rf_off();
		nrf.rf_on(true);
		systimer->delayms(50);
		int64_t timeout = systimer->gettime() + 50000;
		while (systimer->gettime() < timeout)
		{
			if (irq->read() == false)
			{
				uint8_t data[32];
				nrf.read_rx(data, 32);
				
				// version 0 binding request ?
				binding_pkt *pkt = (binding_pkt*)data;
				if (pkt->magic[0] == 'B' && pkt->magic[1] == 'D' && pkt->version == 0 && pkt->crc == uint8_t(crc32(0, data+3, 29)))
				{
					// check payload is ack?
					binding_info_v0 *payload = (binding_info_v0 *)pkt->payload;
					
					if (payload->cmd == cmd_ack_binding)
					{
						dbg2->write(true);
						return 0;
					}
				}				
			}
			dbg2->write(systimer->gettime() % 250000 < 125000);
		}
	}

	return -1;
}

int main()
{
	space_init();	
	board_init();
	seed = board_get_seed();
	
	irq->set_mode(MODE_IN);
	dbg->set_mode(MODE_OUT_PushPull);
	dbg->write(false);
	dbg2->set_mode(MODE_OUT_PushPull);
	dbg2->write(true);
	
	hoop_id = 0;
	int c = 1;
	while( (c = nrf.init(spi, cs, ce)) != 0)
	{
		dbg2->write(false);
		systimer->delayms(100);
		dbg2->write(true);
		systimer->delayms(100);
	}
	
	if (bind_button && bind_button->read() == false)
		binding_loop();
	rando.set_seed(seed);
	rando.reset(0);
	uint64_t key4[4] = {seed, seed, seed, seed};
	aes.set_key((uint8_t*)key4, 256);
	nrf.rf_off();
	nrf.set_tx_address((uint8_t*)&seed, 3);
	nrf.rf_on(false);
	interrupt->set_callback(nrf_irq_entry, NULL);
		
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
	
	timer->set_callback(timer_entry, NULL);
	timer->set_period(hoop_interval);
	
	int64_t lt = systimer->gettime();
	
	int16_t v[6] = {0};
	dbg2->write(false);
	while(1)
	{
	}
}
