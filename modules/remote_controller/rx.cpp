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
#include "packet_format.h"

#include <Protocol/sbus.h>

// BSP
using namespace HAL;
using namespace devices;

// parameters
uint64_t seed = 0x1234567890345678;
bool enable_telemetry = false;


nrf_pkt uplink_pkt;
uplink_payload_v0 *uplink_payload = (uplink_payload_v0 *)uplink_pkt.payload;
nrf_pkt downlink_pkt;
downlink_payload_v0 *downlink_payload = (downlink_payload_v0 *)downlink_pkt.payload;

AESCryptor2 aes;
OLED96 oled;
int o = 0;
int rdp;
int hoop_id = -1;
int trx_hoop_id = -1;
int miss = 99999;
int maxmiss = 0;
int packet_time = 1363;		// in us
int rf_latency = 45;
int rx_process_time = 208;	// 72Mhz STM32F1 -O0, will be updated in IRQ
int spi_tune_time = 60;
int telemetry_id = 0;
int telemetry_ack_count = 0;
int telemetry_nack_count = 0;
int telemetry_nack2_count = 0;
int telemetry_down_bytes = 0;
int telemetry_up_bytes = 0;


bool downlink_ready = true;
int last_uplink_telemetry_id = -1;

int64_t last_valid_packet = -1000000;
uint16_t hoop_interval = 1000;
uint32_t randomizing[4] = {0, 0, 0, 0};

class NRF24L01_ANT : public NRF24L01
{
public:
	virtual int rf_on(bool rx)
	{
		select_ant(randomizing, true);
		return NRF24L01::rf_on(rx);
	}
} nrf;

void encrypt_downlink_pkt()
{

}

void fill_telemetry_pkt()
{
	static uint8_t buf[32] = "HelloWorld";
	static int buf_size = 10;
	if (downlink_ready && telemetry)
	{
		buf_size = telemetry->read(buf, sizeof(uplink_payload->telemetry));
		if (buf_size > 0)
		{
			telemetry_id ++;
			telemetry_id &= 0xf;
			downlink_ready = false;
			telemetry_down_bytes += buf_size;
		}
		else
		{
			buf_size = 0;
		}
	}

	downlink_payload->telemetry_id = telemetry_id;
	memcpy(downlink_payload->telemetry, buf, buf_size);
	downlink_payload->ack_id = uplink_payload->telemetry_id;
	downlink_payload->telemetry_size = buf_size;

	downlink_pkt.crc = crc32(0, &downlink_pkt, 30);
	uint8_t *p = (uint8_t*)&downlink_pkt;
	aes.encrypt(p, p);
	aes.encrypt(p+16, p+16);
}

uint32_t pos2rando(int pos)
{
	randomizing[0] = pos;
	randomizing[1] = 0;
	randomizing[2] = 0;
	randomizing[3] = 0;
	aes.encrypt((uint8_t*)randomizing, (uint8_t*)randomizing);
	return randomizing[0];
}

bool LOS()
{
	if (hoop_interval == 1000)
		return miss > 1000;
	else
		return miss > 500;
}

int tx_spi_time = 0;
int hoop_to(int next_hoop_id)
{	
	int channel = ((pos2rando(next_hoop_id) & 0xffff) * 100) >> 16;	
	hoop_id = next_hoop_id;
	if (next_hoop_id == trx_hoop_id && ce->read()) 
		return 0;
	
	nrf.rf_off();	
	
	// 68
	nrf.write_reg(RF_CH, channel);
	
	if (!(hoop_id & 1) || LOS() || !enable_telemetry)
	{
		nrf.rf_on(true);
	}
	else
	{
		// 131
		fill_telemetry_pkt();
		
		//if(!downlink_ready)
		{
			int64_t t = systimer->gettime();
			// 105
			nrf.write_tx((uint8_t*)&downlink_pkt, 32);
			nrf.rf_on(false);

			tx_spi_time = systimer->gettime() - t;
		}
	}
	
	return 0;
}

void nrf_irq_entry(void *parameter, int flags)
{
	int64_t ts = systimer->gettime();
	timer->disable_cb();
	nrf.rf_off();
	nrf.write_reg(7, nrf.read_reg(7));
	trx_hoop_id = -1;
	
	int fifo_state = nrf.read_reg(FIFO_STATUS);
	int next_hoop_id = -1;
	while ((fifo_state&1) == 0)
	{
		nrf_pkt data;
		nrf.read_rx((uint8_t*)&data, 32);
		rdp = nrf.read_reg(9);
		fifo_state = nrf.read_reg(FIFO_STATUS);
		
		aes.decrypt((uint8_t*)&data, (uint8_t*)&data);
		aes.decrypt((uint8_t*)&data+16, (uint8_t*)&data+16);
		uint16_t crc_calc = crc32(0, (uint8_t*)&data, 30);

		if (crc_calc != data.crc)
			continue;
		
		dbg->write(false);
		o++;
		next_hoop_id = (data.hoop_id + 1)&0xffff;
		memcpy(&uplink_pkt, &data, 32);
		last_valid_packet = ts;

		// telemetry
		if (uplink_payload->ack_id == telemetry_id)
		{
			telemetry_ack_count ++;
			downlink_ready = true;
		}
		else
		{
			int m1 = telemetry_id - 1;
			if (m1 == -1)
				m1 = 15;
			
			if (uplink_payload->ack_id == m1)
				telemetry_nack_count ++;
			else
				telemetry_nack2_count ++;				
		}

		if (uplink_payload->telemetry_id != last_uplink_telemetry_id)
		{
			last_uplink_telemetry_id = uplink_payload->telemetry_id;

			if (telemetry)
				telemetry->write(uplink_payload->telemetry, uplink_payload->telemetry_size);		
		}
	}
	
	
	if (next_hoop_id >= 0)
	{
		timer->restart();
		miss = 0;
		hoop_to(next_hoop_id);
		rx_process_time = systimer->gettime() - ts;
	}
	else
	{
		trx_hoop_id = hoop_id + 1;
		int channel = ((pos2rando(hoop_id+1) & 0xffff) * 100) >> 16;	
		nrf.write_reg(RF_CH, channel);
		nrf.rf_on(true);
	}
	
	timer->enable_cb();
	dbg->write(true);
}

void timer_entry(void * p)
{
	interrupt->disable();
	bool is_rx = !(hoop_id & 1) || LOS() || !enable_telemetry;
	
	if (is_rx)
		miss ++;
	
	if (!LOS() && is_rx)
		dbg2->write(false);
	// hoop slower when lost sync
	if (!LOS() || miss % 50 == 0)
		hoop_to((hoop_id+1)&0xffff);
	if (!LOS() && is_rx)
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
	bool new_enable_telemetry;
	while(systimer->gettime() < timeout)
	{
		watchdog_reset();

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
				new_enable_telemetry = payload->enable_telemetry && telemetry != NULL;
				
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
			enable_telemetry = new_enable_telemetry;
			space_write("tele", 4, &enable_telemetry, 1, NULL);
			
			// ACK packets
			binding_pkt pkt = {{'B','D'}, 0, 0};
			binding_info_v0 *payload = (binding_info_v0 *)pkt.payload;
			payload->cmd = cmd_ack_binding;
			payload->enable_telemetry = telemetry != NULL;
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

void carrier_test()
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
}
	int lp = 0;

int main()
{
	space_init();
	space_read("seed", 4, &seed, 8, NULL);
	space_read("tele", 4, &enable_telemetry, 1, NULL);
	board_init();
	watchdog_init();
	dbg->write(true);
	dbg2->write(true);
	dbg->set_mode(MODE_OUT_OpenDrain);
	dbg2->set_mode(MODE_OUT_OpenDrain);

	memset(&downlink_pkt, 0, sizeof(downlink_pkt));

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
	nrf.set_tx_address((uint8_t*)&seed, 3);
	nrf.set_rx_address(0, (uint8_t*)&seed, 3);
	nrf.enable_rx_address(0, true);
	nrf.write_reg(RF_CH, 95);
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
	packet_time = nrf.is_bk5811() ? 450 : 1373;
	
	interrupt->set_callback(nrf_irq_entry, NULL);
	timer->set_callback(timer_entry, NULL);
	timer->set_period(hoop_interval);
	timer->set_priority(3);
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	dbg->write(true);
	nrf.rf_on(true);


	if (ebus)
		ebus->set_baudrate(115200);
	if (sbus)
		sbus->set_baudrate(100000);
	
	int lo = 0;
	int64_t t = systimer->gettime();
	int64_t lt = t;
	while(1)
	{
		custom_output(uplink_pkt.payload, 28, systimer->gettime() - last_valid_packet);
		
		static int64_t last_ppm_out = systimer->gettime();
		static int64_t last_sbus_out = systimer->gettime();
		static int64_t last_ebus_out = systimer->gettime();

		
		int16_t data[6];
		for(int i=0; i<6; i++)
			data[i] = uplink_payload->channel_data[i]* 1000 / 4096 + 1000;
		if (miss > 2000 || systimer->gettime() > last_valid_packet + 2000000)
			data[2] = 800;

		if (ebus && systimer->gettime() - last_ebus_out > 10000)
		{
			last_ebus_out = systimer->gettime();
			int8_t ebus_frame[15] = {0};
			ebus_frame[0] = 0x85;
			ebus_frame[1] = 0xA3;
								
			memcpy(ebus_frame+2, uplink_payload->channel_data, 12);
			
			if (miss > 200)
				((int16_t*)(ebus_frame+2))[2] = -1000;
			ebus_frame[14] = crc32(0, ebus_frame+2, 12);
			
			ebus->write(ebus_frame, sizeof(ebus_frame));
		}

		if (sbus && last_valid_packet - last_sbus_out > 10000)
		{
			last_sbus_out = last_valid_packet;

			sbus_u s = {0x0f};
			uint8_t key = uplink_payload->keys[0];
			
			s.dat.chan2 = (((int)uplink_payload->channel_data[0] * 800) >> 11)+200;	// roll
			s.dat.chan1 = (((int)uplink_payload->channel_data[1] * 800) >> 11)+200;	// pitch
			s.dat.chan3 = (((int)uplink_payload->channel_data[2] * 800) >> 11)+200;	// throttle
			s.dat.chan4 = (((int)uplink_payload->channel_data[3] * 800) >> 11)+200;	// yaw
			s.dat.chan5 = ((((int)uplink_payload->channel_data[4] * 800) >> 11))+200;	// mode
			s.dat.chan6 = ((((int)uplink_payload->channel_data[5] * 800) >> 11))+200;	// arm

			s.dat.chan8 = !(key&2) ? 1800 : 200;				// aux4 stop
			s.dat.chan9 = !(key&8) ? 1024 : 200;				// aux5 key
			s.dat.chan10 = !(key&16) ? 1024 : 200;				// aux6 key

			sbus->write(&s, sizeof(s));
		}

		if (ppm && systimer->gettime() - last_ppm_out > 20000)
		{
			last_ppm_out = systimer->gettime();
			ppm->write(data, 6, 0);
		}
		
		if (SCL&&SDA)
		{
			char tmp[100];
			sprintf(tmp, "%dp, pos=%d    ", o, *(uint16_t*)uplink_pkt.hoop_id);
			oled.show_str(0, 0, tmp);
			sprintf(tmp, "rdp=%d,%dp/s, %dms    ", rdp, lp, maxmiss * hoop_interval / 1000);
			oled.show_str(0, 1, tmp);
			
			int64_t current = systimer->gettime();
			
			int16_t *channel = uplink_payload->channel_data;
			
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
		
		watchdog_reset();
	}
}
