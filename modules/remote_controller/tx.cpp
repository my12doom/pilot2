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
#include "packet_format.h"

// BSP
using namespace HAL;
using namespace devices;

// parameters
uint64_t seed = 0x1234567890345678;
configure_entry config[6];
configure_entry channel_statics[6];
bool enable_telemetry = false;
//uint8_t ch_high = 125;
//uint8_t ch_low = 106;

uint8_t ch_high = 100;
uint8_t ch_low = 0;

nrf_pkt uplink_pkt;
uplink_payload_v0 *uplink_payload = (uplink_payload_v0*)uplink_pkt.payload;
nrf_pkt downlink_pkt;
downlink_payload_v0 *downlink_payload = (downlink_payload_v0*)downlink_pkt.payload;
int telemetry_id = 0;
int telemetry_down_bytes = 0;
int last_downlink_telemetry_id = -1;
int telemetry_up_bytes = 0;
bool telemetry_uplink_ready = true;
bool got_downlink = false;

uint16_t hoop_id = 0;
uint16_t hoop_interval = 1000;
int64_t ts;
int dt;
int rdp;
AESCryptor2 aes;
uint32_t randomizing[4] = {0, 0, 0, 0};

int packet_time = 1363;		// in us
int rf_latency = 45;
int rx_pkt = 0;
int rx_pkt_s = 0;
int rx_pkt_s_lpf = 0;

class NRF24L01_ANT : public NRF24L01
{
public:
	virtual int rf_on(bool rx)
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
	nrf.write_reg(7, nrf.read_reg(7));				// sending 32bytes payload with 3byte address and 2byte CRC cost ~1363us
													// delay between tx and rx is 25us max
		
	// read out downlink packets
	//ts = systimer->gettime();
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	while ((fifo_state&1) == 0)
	{
		nrf_pkt pkt;
		uint8_t *p = (uint8_t*)&pkt;
		nrf.read_rx(p, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		
		aes.decrypt(p, p);
		aes.decrypt(p+16, p+16);
		uint16_t crc_calc = crc32(0, p, 30);
		
		if (pkt.crc == crc_calc)
		{
			downlink_pkt = pkt;
			rx_pkt ++;

			if (downlink_led)
				downlink_led->write(0);
			
			got_downlink = true;

			// handle telemetry
			if (downlink_payload->ack_id == telemetry_id)
				telemetry_uplink_ready = true;
			if (downlink_payload->telemetry_id != last_downlink_telemetry_id)
			{
				last_downlink_telemetry_id = downlink_payload->telemetry_id;

				if (telemetry)
				{
					telemetry->write(downlink_payload->telemetry, downlink_payload->telemetry_size);
					telemetry_down_bytes += downlink_payload->telemetry_size;
				}
			}
		}
	}

	// listen for downlink
	if (enable_telemetry && (hoop_id&1))
	{
		int channel = (((pos2rando(hoop_id) & 0xffff) * (ch_high-ch_low)) >> 16) + ch_low;
		nrf.write_reg(5, channel);
		nrf.rf_on(true);
		
		// dirty fix for BK5811
		if (hoop_interval == 1000)
		{
			nrf.rf_off();
			nrf.rf_on(true);
		}
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

void apply_calibration(int16_t *data, int count)
{
	memcpy(channel_data, data, count*2);
	for(int i=0; i<count; i++)
	{
		int16_t d = data[i];

		// update channel statics
		if (d < channel_statics[i]._min)
			channel_statics[i]._min = d;
		if (d > channel_statics[i]._max)
			channel_statics[i]._max = d;

		// apply calibration
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

void fill_telemetry()
{
	static uint8_t buf[32] = "HelloWorld";
	static int buf_size = 10;
	if (telemetry_uplink_ready && telemetry)
	{
		buf_size = telemetry->read(buf, sizeof(uplink_payload->telemetry));
		if (buf_size > 0)
		{
			telemetry_id ++;
			telemetry_id &= 0xf;
			telemetry_uplink_ready = false;
			telemetry_up_bytes += buf_size;
		}
		else
		{
			buf_size = 0;
		}
	}

	uplink_payload->telemetry_id = telemetry_id;
	memcpy(uplink_payload->telemetry, buf, buf_size);
	uplink_payload->telemetry_size = buf_size;
}

int tx_tx_process_time = 0;
void timer_entry(void *p)
{
	interrupt->disable();
	dbg->write(true);
	
	uplink_pkt.hoop_id = hoop_id;
	int channel = (((pos2rando(hoop_id) & 0xffff) * (ch_high-ch_low)) >> 16) + ch_low;
	hoop_id ++;
	
	if (!enable_telemetry || (hoop_id& 1))
	{
		nrf.rf_off();
		nrf.write_reg(5, channel);
		int64_t t = systimer->gettime();
		read_channels(uplink_payload->channel_data, 6);
		apply_calibration(uplink_payload->channel_data, 6);
		read_keys(uplink_payload->keys, 2);

		fill_telemetry();
		
		uplink_payload->ack_id = downlink_payload->telemetry_id;
		uplink_payload->telemetry_id = telemetry_id;

		uplink_pkt.crc = crc32(0, &uplink_pkt, 30);		
		uint8_t *p = (uint8_t*)&uplink_pkt;
		aes.encrypt(p, p);
		aes.encrypt(p+16, p+16);
			
		
		nrf.write_tx(p, 32);
		nrf.rf_on(false);
		tx_tx_process_time = systimer->gettime() - t;
		
		if (downlink_led && !got_downlink)
			downlink_led->write(enable_telemetry);
		
		rx_pkt_s_lpf = (rx_pkt_s_lpf * 235 + (got_downlink ? 10000 : 0) * 21) >> 8;
		got_downlink = false;
	}
	
	interrupt->enable();
	ts = systimer->gettime();
	dbg->write(false);
}

int binding_loop()
{
	if (downlink_led)
		downlink_led->write(1);

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
	payload->enable_telemetry = telemetry != NULL;
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
				binding_pkt *rx_pkt = (binding_pkt*)data;
				if (rx_pkt->magic[0] == 'B' && rx_pkt->magic[1] == 'D' && rx_pkt->version == 0 && rx_pkt->crc == uint8_t(crc32(0, data+3, 29)))
				{
					// check payload is ack?
					binding_info_v0 *rx_payload = (binding_info_v0 *)rx_pkt->payload;
					
					if (rx_payload->cmd == cmd_ack_binding)
					{
						dbg2->write(true);
						
						memcpy(&seed, rx_payload->key, 8);
						seed ^= board_get_seed();
						space_write("seed", 4, &seed, 8, NULL);

						enable_telemetry = payload->enable_telemetry && rx_payload->enable_telemetry;
						space_write("tele", 4, &enable_telemetry, 1, NULL);

						if (vibrator)
							vibrator->write(false);

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
		
		watchdog_reset();
		
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
	nrf.set_rx_address(0, (uint8_t*)&seed, 3);
	nrf.enable_rx_address(0, true);
	nrf.enable_rx_address(1, false);
	nrf.write_cmd(FLUSH_TX, NOP);
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(7, nrf.read_reg(7));
	nrf.rf_on(false);
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_priority(2);
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
		watchdog_reset();
	}
}

int reset_channel_statics()
{
	for(int i=0; i<sizeof(channel_statics)/sizeof(channel_statics[0]); i++)
	{
		channel_statics[i]._min = 2048;
		channel_statics[i]._max = 2048;
		channel_statics[i].middle = 2048;
	}

	return 0;
}

int apply_channel_statics()
{
	for(int i=0; i<sizeof(channel_statics)/sizeof(channel_statics[0]); i++)
	{
		config[i]._min = channel_statics[i]._min;
		config[i]._max = channel_statics[i]._max;
		config[i].middle = channel_data[i];
	}

	space_write("conf", 4, &config, sizeof(config), NULL);
	return 0;
}

int main()
{
	is_tx = true;
	space_init();	
	space_read("seed", 4, &seed, 8, NULL);
	space_read("tele", 4, &enable_telemetry, 1, NULL);
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
	watchdog_init();
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
	packet_time = nrf.is_bk5811() ? 450 : 1373;
	
	//if (seed == 0x1234567890345678)
	//	binding_loop();
	//carrier_test();
	tx_on();
	reset_channel_statics();
	
	bool last_bind_button = false;
	if (bind_button)
		last_bind_button = bind_button->read();
	int64_t last_key_down = 0;
	int streak = 0;
	int64_t last_pkt_counting = 0;
	int last_pkt_count = 0;
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
		
		if (systimer->gettime() > last_pkt_counting + 1000000)
		{
			rx_pkt_s = rx_pkt - last_pkt_count;
			last_pkt_count = rx_pkt;
			last_pkt_counting = systimer->gettime();
		}
		
		watchdog_reset();
	}
}
