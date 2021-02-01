#include <HAL/Interface/ISysTimer.h>
#include <HAL/Interface/IUART.h>
#include "board.h"
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <HAL/aux_devices/NRF24L01.h>
#include <utils/space.h>
#include <utils/RIJNDAEL.h>
#include <utils/AES.h>
#include "binding.h"

// BSP
using namespace HAL;
using namespace devices;

uint8_t data[32];
uint8_t rx_data[32];
uint16_t hoop_id = 0;
uint64_t seed = 0x1234567890345678;
uint16_t hoop_interval = 1000;
int64_t ts;
int dt;
int rdp;
AESCryptor2 aes;
uint32_t randomizing[4] = {0, 0, 0, 0};
configure_entry config[6];

class NRF24L01_ANT : public NRF24L01
{
public:
	int rf_on(bool rx)
	{
		select_ant(randomizing, true);
		return NRF24L01::rf_on(rx);
	}
} nrf;

uint32_t pos2rando(int pos)
{
	randomizing[0] = pos;
	randomizing[1] = 0;
	randomizing[2] = 0;
	randomizing[3] = 0;
	aes.encrypt((uint8_t*)randomizing, (uint8_t*)randomizing);
	return randomizing[0];
}

void nrf_irq_entry(void *parameter, int flags)
{
	timer->disable_cb();
	dbg2->write(false);
		
	// clear all interrupts
	nrf.rf_off();
	nrf.write_reg(7, nrf.read_reg(7));				// sending 32bytes payload with 3byte address and 2byte CRC cost ~1373us
													// delay between tx and rx is 25us max
		
	// read out RX packets
	//ts = systimer->gettime();
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	while ((fifo_state&1) == 0)
	{
		nrf.read_rx(data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
	}
	
	//dt = systimer->gettime() - ts;
	dbg2->write(true);
	timer->enable_cb();
}

static int iabs(int a)
{
	return a>0?a:-a;
}

int16_t channel_data[6];
int16_t channel_data_o[6];

void process_channels(int16_t *data, int count)
{
	memcpy(channel_data, data, count*2);
	for(int i=0; i<count; i++)
	{
		int16_t d = data[i];
		if (iabs(d-config[i].middle) < config[i].dead_band)
			d = 2048;
		else if (d <= config[i].middle)
		{
			d = 2048 - (config[i].middle-d) * 2048 / (config[i].middle - config[i]._min);
		}
		
		else if (d > config[i].middle)
		{
			d = (d-config[i].middle) * 2048 / (config[i]._max - config[i].middle) + 2048;
		}
		if (d > 4095)
			d = 4095;
		if (d < 0)
			d = 0;
		
		if (config[i].reverse)
			d = 4095-d;
		
		data[i] = d;
	}
	memcpy(channel_data_o, data, count*2);
	
}

void timer_entry(void *p)
{
	interrupt->disable();
	dbg->write(true);
	
	*(uint16_t*)data = hoop_id;	
	int channel = ((pos2rando(hoop_id) & 0xffff) * 100) >> 16;
	hoop_id ++;
	//channel = hoop_id%100;
	nrf.rf_off();
	nrf.write_reg(5, channel);
	
	//if (channel < 10)
	if (1)
	{		
		read_channels((int16_t*)(data+2), 6);
		process_channels((int16_t*)(data+2), 6);
		read_keys(data+14, 8);
		
		*(uint16_t*)(data+30) = crc32(0, data, 30);
		
		aes.encrypt(data, data);
		aes.encrypt(data+16, data+16);
			
		nrf.write_tx(data, 32);
		nrf.rf_on(false);
	}
	else
	{
		nrf.rf_on(true);
	}
	
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
	uint64_t board_seed = board_get_seed();
	memcpy(payload->key, &board_seed, 8);
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
		systimer->delayms(5);
		int64_t timeout = systimer->gettime() + 95000;
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
						
						memcpy(&seed, payload->key, 8);
						seed ^= board_get_seed();
						space_write("seed", 4, &seed, 8, NULL);

						return 0;
					}
				}				
			}
			dbg2->write(systimer->gettime() % 500000 < 250000);
			if (vibrator)
			{
				int frac = systimer->gettime() % 750000;
				vibrator->write(frac<125000 || (frac > 250000 && frac < 375000));
			}
		}
		
	}

	if (vibrator)
		vibrator->write(false);
	
	return -1;
}

void tx_on()
{
	hoop_id = 0;
	uint64_t key4[4] = {seed, seed, seed, seed};
	aes.set_key((uint8_t*)key4, 256);
	nrf.rf_off();
	nrf.set_tx_address((uint8_t*)&seed, 3);
	nrf.write_cmd(FLUSH_TX, NOP);
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(7, nrf.read_reg(7));
	nrf.rf_on(false);
	interrupt->set_callback(nrf_irq_entry, NULL);	
	timer->set_period(hoop_interval);
	timer->set_callback(timer_entry, NULL);	
		
	dbg2->write(false);
}

void tx_off()
{
	interrupt->set_callback(NULL, NULL);	
	timer->set_callback(NULL, NULL);	
	nrf.rf_off();
	
	dbg2->write(true);
}

int carrier_test()
{
	nrf.rf_off();
	systimer->delayms(2);
	nrf.bk5811_carrier_test(true);
	nrf.write_reg(RF_SETUP, 0x96);
	nrf.write_reg(RF_CH, 1);
	nrf.rf_on(false);
	
	while(1)
	{
	}
	return 0;
}

int main()
{
	space_init();	
	space_read("seed", 4, &seed, 8, NULL);
	if (space_read("conf", 4, &config, sizeof(config), NULL) < 0)
	{
		// default configuration
		for(int i=0; i<sizeof(config)/sizeof(config[0]); i++)
		{
			config[i]._min = 0;
			config[i]._max = 4095;
			config[i].middle = 2048;
			config[i].reverse = 0;
			config[i].dead_band = 0;
		}

		space_write("conf", 4, &config, sizeof(config), NULL);
	}

	// don't do flash erasure after board init
	// may corrupt adc dma transfer 
	board_init();
	
	// read config again, if board modified
	space_read("conf", 4, &config, sizeof(config), NULL);
	
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
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;	
	
	//if (seed == 0x1234567890345678)
	//	binding_loop();
	//carrier_test();
	tx_on();
	
	bool last_bind_button = false;
	if (bind_button)
		last_bind_button = bind_button->read();
	int64_t last_key_down = 0;
	int streak = 0;
	while(1)
	{
		if (bind_button)
		{
			bool b = bind_button->read();
			if (b && !last_bind_button)
			{
				if (systimer->gettime() - last_key_down < 500000)
					streak ++;
				else
				{
					streak = 1;
				}
				
				last_key_down = systimer->gettime();
				
				if (streak > 5)
				{
					tx_off();
					binding_loop();
					tx_on();
				}
			}
			
			last_bind_button = b;
		}
		
	}
}
