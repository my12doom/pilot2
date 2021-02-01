#include <HAL/Interface/ISysTimer.h>
#include <HAL/Interface/IUART.h>
#include "board.h"
#include <utils/log.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <HAL/aux_devices/NRF24L01.h>
#include <HAL/aux_devices/OLED_I2C.h>
#include <Protocol/crc32.h>
#include <utils/RIJNDAEL.h>
#include <utils/AES.h>
#include <utils/space.h>
#include "binding.h"

// BSP
using namespace HAL;
using namespace devices;

uint8_t valid_data[32];
AESCryptor2 aes;
uint64_t seed = 0x1234567890345678;
OLED96 oled;
int o = 0;
int rdp;
int hoop_id = 0;
int miss = 99999;
int maxmiss = 0;
int packet_time = 1373;		// in us
int64_t last_valid_packet = -1000000;
uint16_t hoop_interval = 1000;
uint32_t randomizing[4] = {0, 0, 0, 0};

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

int hoop_to(int next_hoop_id)
{
	nrf.rf_off();
	
	hoop_id = next_hoop_id;
	
	int channel = ((pos2rando(next_hoop_id) & 0xffff) * 100) >> 16;
	
	nrf.write_reg(RF_CH, channel);
	
	//if (channel < 10)
	if (1)
	{	
		nrf.rf_on(true);
	}
	else
	{
		uint8_t pkt[32] = "HelloWorld";
		nrf.write_tx(pkt, 32);
		nrf.rf_on(false);		
	}
	
	return 0;
}

void nrf_irq_entry(void *parameter, int flags)
{
	int64_t ts = systimer->gettime();
	timer->disable_cb();
	nrf.rf_off();
	nrf.write_reg(7, nrf.read_reg(7));
	
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	int next_hoop_id = -1;
	uint8_t data[32];
	while ((fifo_state&1) == 0)
	{
		nrf.read_rx(data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		
		aes.decrypt(data, data);
		aes.decrypt(data+16, data+16);
		uint16_t crc_calc = crc32(0, data, 30);

		if (crc_calc != *(uint16_t*)(data+30))
			continue;
		
		dbg->write(false);
		o++;
		next_hoop_id = *(uint16_t*)data;
		memcpy(valid_data, data, 32);
		last_valid_packet = ts;
	}
	
	if (next_hoop_id >= 0)
	{
		hoop_to((next_hoop_id+1)&0xffff);
		miss = 0;
		int dt = systimer->gettime() - ts;
		int ss = hoop_interval - dt - packet_time-200;
		if (ss > 0)
			systimer->delayus(ss);
		timer->restart();
	}
	
	timer->enable_cb();
	dbg->write(true);
}

void timer_entry(void * p)
{
	interrupt->disable();
	
	miss ++;
	
	if (miss < 2000)
		dbg2->write(false);
	// hoop slower when lost sync
	if (miss < 2000 || miss % 50 == 0)
		hoop_to((hoop_id+1)&0xffff);
	if (miss < 2000)
		dbg2->write(true);
	
	if (maxmiss < miss)
		maxmiss = miss;
	
	interrupt->enable();
}

int binding_loop()
{
	// enable binding address and enter biding channel.
	nrf.rf_off();
	nrf.set_rx_address(0, (uint8_t*)&seed, 3);
	nrf.set_rx_address(1, (uint8_t*)BINDING_ADDRESS, 3);
	nrf.enable_rx_address(0, true);
	nrf.enable_rx_address(1, true);
	nrf.set_tx_address(BINDING_ADDRESS, 3);
	nrf.write_reg(RF_CH, BINDING_CHANNEL);
	nrf.rf_on(true);
	
	int tx_channel_data = 0;
	bool binding_done = false;
	uint64_t new_seed = 0;
	int64_t timeout = systimer->gettime() + 5000000;
	while(systimer->gettime() < timeout)
	{
		// read new packet
		if (irq->read() == false)
		{
			uint8_t data[32];
			nrf.read_rx(data, 32);
			
			// version 0 binding request ?
			binding_pkt *pkt = (binding_pkt*)data;
			if (pkt->magic[0] == 'B' && pkt->magic[1] == 'D' && pkt->version == 0 && pkt->crc == uint8_t(crc32(0, data+3, 29)))
			{
				// check payload
				binding_info_v0 *payload = (binding_info_v0 *)pkt->payload;
				
				if (payload->cmd == cmd_requesting_binding)
				{
					binding_done = true;
					memcpy(&new_seed, payload->key, 8);
					new_seed ^= board_get_seed();
				}
			}
			
			// tx channel data ?
			else
			{
				aes.decrypt(data, data);
				aes.decrypt(data+16, data+16);
				uint16_t crc_calc = crc32(0, data, 30);
				if (crc_calc == *(uint16_t*)(data+30))
					tx_channel_data ++ ;
			}
		}
		
		dbg->write(systimer->gettime() % 500000 > 2500);
		
		// if binding done, save seed and other settings, send ACK packets, and exit 
		if (binding_done)
		{
			// save settings
			seed = new_seed;
			space_write("seed", 4, &seed, 8, NULL);
			
			// ACK packets
			binding_pkt pkt = {{'B','D'}, 0, 0};
			binding_info_v0 *payload = (binding_info_v0 *)pkt.payload;
			payload->cmd = cmd_ack_binding;
			uint64_t ack_seed = board_get_seed();
			memcpy(payload->key, &ack_seed, 8);
			pkt.crc = crc32(0, ((uint8_t*)&pkt)+3, 29);
			
			nrf.rf_off();
			nrf.rf_on(false);
			int64_t t = systimer->gettime();
			dbg->write(false);
			while(systimer->gettime() < t+500000)
			{
				nrf.write_tx((uint8_t*)&pkt, 32);
				
				systimer->delayms(2);
			}
			dbg->write(true);
			
			// clear up and exit
			nrf.rf_off();
			nrf.enable_rx_address(1, false);
			return 0;
		}

		// exit if several channel data packet recieved
		if (tx_channel_data > 3)
		{
			nrf.rf_off();
			nrf.enable_rx_address(1, false);
			return 0;
		}
	}
	
	return 0;
}
int carrier_test()
{
	nrf.rf_off();
	systimer->delayms(2);
	nrf.bk5811_carrier_test(true);
	nrf.write_reg(RF_SETUP, 0x96);
	nrf.write_reg(RF_CH, 0);
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
	board_init();
	dbg->write(true);
	dbg2->write(true);
	dbg->set_mode(MODE_OUT_OpenDrain);
	dbg2->set_mode(MODE_OUT_OpenDrain);
	crc32(0, valid_data, 30);	// to build crc32 lookup table;
	
	if (SCL && SDA)
	{
		I2C_SW i2c(SCL, SDA);
		oled.init(&i2c, 0x78);	
		oled.show_str(0, 0, "init NRF...");
	}
	
	irq->set_mode(MODE_IN);
	
	while(nrf.init(spi, cs, ce) != 0)
		;
	//carrier_test();
	if (SCL && SDA)
		oled.show_str(0, 0, "binding ...");
	uint64_t key4[4] = {seed, seed, seed, seed};
	aes.set_key((uint8_t*)key4, 256);
	binding_loop();
	uint64_t key4after[4] = {seed, seed, seed, seed};
	aes.set_key((uint8_t*)key4after, 256);
	nrf.rf_off();
	nrf.set_rx_address(0, (uint8_t*)&seed, 3);
	nrf.enable_rx_address(0, true);
	nrf.write_reg(RF_CH, 95);
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
	packet_time = nrf.is_bk5811() ? 450 : 1373;
	
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_callback(timer_entry, NULL);
	timer->set_period(hoop_interval);
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	dbg->write(true);
	nrf.rf_on(true);
	
	int lo = 0;
	int64_t t = systimer->gettime();
	int64_t lt = t;
	int lp = 0;
	while(1)
	{
		custom_output(valid_data+2, 28, systimer->gettime() - last_valid_packet);
		
		if (ppm)
		{
			static int64_t last_out = systimer->gettime();
			
			int16_t data[6];
			for(int i=0; i<6; i++)
				data[i] = ((int16_t *)valid_data)[i+1]* 1000 / 4096 + 1000;
			
			if (systimer->gettime() - last_out > 20000)
			{
				last_out = systimer->gettime();
				
				if (miss > 2000 || systimer->gettime() > last_valid_packet + 2000000)
					data[2] = 800;
				ppm->write(data, 6, 0);
				
				if (uart)
				{
					int8_t ebus_frame[15] = {0};
					ebus_frame[0] = 0x85;
					ebus_frame[1] = 0xA3;
										
					memcpy(ebus_frame+2, valid_data+2, 12);
					
					if (miss > 200)
						((int16_t*)(ebus_frame+2))[2] = -1000;
					ebus_frame[14] = crc32(0, ebus_frame+2, 12);
					
					uart->write(ebus_frame, sizeof(ebus_frame));
				}
			}
			
			continue;
		}
		
		if (SCL&&SDA)
		{
			char tmp[100];
			sprintf(tmp, "%dp, pos=%d    ", o, *(uint16_t*)valid_data);
			oled.show_str(0, 0, tmp);
			sprintf(tmp, "rdp=%d,%dp/s, %dms    ", rdp, lp, maxmiss * hoop_interval / 1000);
			oled.show_str(0, 1, tmp);
			
			int64_t current = systimer->gettime();
			
			int16_t *channel = (int16_t *)(valid_data+2);
			
			sprintf(tmp, "%04d,%04d,%04d ", channel[0], channel[1], channel[2]);
			oled.show_str(0, 2, tmp);
			sprintf(tmp, "%04d,%04d,%04d ", channel[3], channel[4], channel[5]);
			oled.show_str(0, 3, tmp);
		}
		
		int64_t current = systimer->gettime();
		if (current - t > 1000000)
		{
			t = current;
			lp = o - lo;
			lo = o;
			maxmiss = 0;
		}
	}
}
