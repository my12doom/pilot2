#include "NRF_RC.h"
#include <math/randomizer.h>
#include <string.h>

#define channel_count 6

using namespace devices;
using namespace HAL;

static int min(int a, int b)
{
	return a > b ? b : a;
}
static int max(int a, int b)
{
	return a > b ? a : b;
}

namespace sensors
{

// statistics functions is mainly for RC calibration purpose.
int NRFIn::get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count)
{
	int count = min(channel_count - start_channel, max_count);
	memcpy(min_out, rc_static[0] + start_channel, count * sizeof(int16_t));
	memcpy(max_out, rc_static[1] + start_channel, count * sizeof(int16_t));
	
	return count;
}
int NRFIn::reset_statistics()
{
	int i;
	for(i=0; i<8; i++)
	{
		rc_static[0][i] = 32767;			// min
		rc_static[1][i] = 0;				// max
	}

	return 0;
}

// total channel count
int NRFIn::get_channel_count()
{
	return channel_count;
}

// return num channel written to out pointer
int NRFIn::get_channel_data(int16_t *out, int start_channel, int max_count)
{
	int count = min(channel_count - start_channel, max_count);
	for(int i=0; i<count; i++)
		out[i] = valid_data[start_channel+i] * 1000 / 4096 + 1000;
		
	return count;
}

// return num channel written to out pointer
int NRFIn::get_channel_update_time(int64_t *out, int start_channel, int max_count)
{
	for(int i=0; i<max_count; i++)
		out[i] = last_packet_time;
	return max_count;
}

HAL::RCIN_State NRFIn::state()
{
	return (systimer->gettime() - last_packet_time > 500000) ? RCIN_Fail : HAL::RCIN_Normal;
}

NRFIn::NRFIn()
{
	o = 0;
}

NRFIn::~NRFIn()
{
}

int NRFIn::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce, HAL::IInterrupt *irq, HAL::ITimer *timer)
{	
	this->ce = ce;
	this->interrupt = irq;
	this->timer = timer;

	rando.set_seed(0x1234567890345678);
	hoop_id = 0;

	for(int i=0; i<10000; i++)
	{
		if(i == 9000)
			return -1;
		if (nrf.init(spi, cs, ce) == 0)
			break;
	}
		

	nrf.write_reg(RF_CH, 95);
	nrf.rf_on(true);
	hoop_interval = nrf.is_bk5811() ? 1000 : 2000;
			
	nrf.write_cmd(FLUSH_RX, NOP);
	nrf.write_reg(STATUS, nrf.read_reg(STATUS));
	
	irq->set_callback(nrf_irq_entry, this);
	timer->set_callback(timer_entry, this);
	timer->set_period(hoop_interval);
	
	return 0;
}

int NRFIn::hoop_to(int next_hoop_id)
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

void NRFIn::nrf_irq_handler()
{
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
		memcpy(valid_data, data+2, sizeof(valid_data));
		last_packet_time = systimer->gettime();
	}
	
	if (next_hoop_id >= 0)
	{
		hoop_to(next_hoop_id+2);
		miss = 0;
		timer->restart();
	}
	
	timer->enable_cb();
}

void NRFIn::timer_handler()
{
	miss ++;
	
	if (miss < 200)
	{
		interrupt->disable();
//		dbg2->write(false);
		if (miss == 0)
			hoop_to((hoop_id+2)%65536);
		else
			hoop_to((hoop_id+1)%65536);
//		dbg2->write(true);
		interrupt->enable();
	}
	
	if (maxmiss < miss)
		maxmiss = miss;
	
}


int NRFIn::get_latency()
{
	int64_t current = systimer->gettime();
	static int64_t t = 0;
	static int lp = 0;
	static int lo = 0;
	
	if (current - t > 1000000)
	{
		t = current;
		lp = o - lo;
		lo = o;
		
		printf("NRF RC:%dp/s, rdp=%d, maxmiss=%d\n", lp, rdp, maxmiss);

		maxmiss = 0;
	}
	
	return maxmiss * hoop_interval;
}

}
