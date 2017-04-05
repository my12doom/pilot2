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
#include "randomizer.h"

// BSP
using namespace HAL;
using namespace devices;

NRF24L01 nrf;

uint8_t valid_data[32];
randomizer<128, 512> rando;
uint64_t seed = 0x1234567890345678;
OLED96 oled;
int o = 0;
int rdp;
int hoop_id = 0;
int miss = 99999;
int maxmiss = 0;
uint16_t hoop_interval = 1000;
HAL::IRCOUT *ppm = NULL;
HAL::IUART *uart = NULL;

int hoop_to(int next_hoop_id)
{
	ce->write(false);
	
	if (next_hoop_id != hoop_id + 1)
		rando.reset(next_hoop_id);
	
	hoop_id = next_hoop_id;
	

	int channel = (int64_t)rando.next() * 100 / 0xffffffff;
	
	nrf.write_reg(RF_CH, channel);
	
	ce->write(true);
	
	return 0;
}

void nrf_irq_entry(void *parameter, int flags)
{
	dbg->write(false);
	timer->disable_cb();
	nrf.write_reg(7, nrf.read_reg(7));
	
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	int next_hoop_id = -1;
	uint8_t data[32];
	while ((fifo_state&1) == 0)
	{
		nrf.read_rx(data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		o++;
		
		next_hoop_id = *(uint16_t*)data;
		memcpy(valid_data, data, 32);
	}
	
	if (next_hoop_id >= 0)
	{
		hoop_to(next_hoop_id+2);
		miss = 0;
		timer->restart();
	}
	
	timer->enable_cb();
	dbg->write(true);
}

void timer_entry(void * p)
{
	interrupt->disable();
	
	miss ++;
	
	if (miss < 200)
	{
		dbg2->write(false);
		if (miss == 0)
			hoop_to((hoop_id+2)%65536);
		else
			hoop_to((hoop_id+1)%65536);
		dbg2->write(true);
	}
	
	if (maxmiss < miss)
		maxmiss = miss;
	
	interrupt->enable();
}

int main()
{		
	board_init();
	I2C_SW i2c(SCL, SDA);
	oled.init(&i2c, 0x78);
	
	oled.show_str(0, 0, "init NRF...");
	
	irq->set_mode(MODE_IN);
	dbg->set_mode(MODE_OUT_OpenDrain);
	dbg2->set_mode(MODE_OUT_OpenDrain);
	dbg->write(true);
	dbg2->write(true);
	
	rando.set_seed(seed);
	while(nrf.init(spi, cs, ce) != 0)
		;
	nrf.write_reg(RF_CH, 95);
	nrf.rf_on(true);
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
	
	for(int i=0; i<30; i++)
	{
		printf("reg(%d)=%02x\n", i, nrf.read_reg(i));
	}
	
	int lo = 0;
	int64_t t = systimer->gettime();
	int64_t lt = t;
	int lp = 0;
	
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	dbg->write(true);
	
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_callback(timer_entry, NULL);
	timer->set_period(hoop_interval);
		
	while(1)
	{
		if (ppm)
		{
			static int64_t last_out = systimer->gettime();
			
			int16_t data[6];
			for(int i=0; i<6; i++)
				data[i] = ((int16_t *)valid_data)[i+1]* 1000 / 4096 + 1000;
			
			if (systimer->gettime() - last_out > 20000)
			{
				last_out = systimer->gettime();
				
				if (miss > 200)
					data[2] = 800;
				ppm->write(data, 6, 0);
				
				if (uart)
				{
					int8_t ebus_frame[15] = {0};
					ebus_frame[0] = 0x85;
					ebus_frame[1] = 0xA3;
					
					int16_t *p = (int16_t*)(ebus_frame+2);
					
					memcpy(p, valid_data, 12);
					
					ebus_frame[14] = crc32(0, ebus_frame+2, 12);
					
					uart->write(ebus_frame, sizeof(ebus_frame));
				}
			}
			
			continue;
		}
		
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
		
		if (current - t > 1000000)
		{
			t = current;
			lp = o - lo;
			lo = o;
			maxmiss = 0;
		}
	}
}
