#include "SX127x.h"
#include <string.h>
#include <stdio.h>
#include <HAL/Interface/ISysTimer.h>

using namespace HAL;

SX127x::SX127x()
{

}

SX127x::~SX127x()
{

}

int SX127x::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *txen, HAL::IGPIO *rxen)
{
	this->spi = spi;
	this->cs = cs;
	this->txen = txen;
	this->rxen = rxen;

	spi->set_speed(12000000);
	spi->set_mode(0, 0);
	cs->set_mode(MODE_OUT_PushPull);
	if (txen)
	{
		txen->set_mode(MODE_OUT_PushPull);
		txen->write(0);
	}
	if (rxen)
	{
		rxen->set_mode(MODE_OUT_PushPull);
		rxen->write(0);
	}

	// chip probe
	char challenge[8] = {0x85, 0xa3, 0x00, 0x7b, 0x66, 0x76, 0x55, 0xaa};
	char challenge_read[8] = {0};
	write_reg(0x0d, 0);
	write_fifo(challenge, sizeof(challenge));
	write_reg(0x0d, 0);
	read_fifo(challenge_read, sizeof(challenge));
	if (memcmp(challenge, challenge_read, sizeof(challenge)))
	{
		printf("SX127x probe failed\n");
		return -1;
	}

	// basic configuration
	set_mode(mode_sleep);
	write_reg(0x01, 0x88);		// enable Lora
	set_tx_power(20);
	set_rate(500000, 1, 7);
	write_reg(0x0e, 0x00);		// disable RSSI smoothing
	set_frequency(439.0);
	set_mode(mode_standby);

	printf("sx1278 init done\n");

	return 0;
}

int SX127x::write_reg(uint8_t reg, uint8_t v)
{
	cs->write(0);
	systimer->delayus(10);
	spi->txrx(0x80 | (reg&0x7f));
	spi->txrx(v);
	cs->write(1);
	return 0;
}

uint8_t SX127x::read_reg(uint8_t reg)
{
	cs->write(0);
	systimer->delayus(10);
	spi->txrx(reg&0x7f);
	reg = spi->txrx(0);
	cs->write(1);
	return reg;
}

int SX127x::write_fifo(const void *buf, int size)
{
	uint8_t dummy[256];
	const uint8_t *data = (const uint8_t*)buf;
	cs->write(0);
	systimer->delayus(10);
	spi->txrx(0x80);
	spi->txrx2((uint8_t*)buf, dummy, size);
	cs->write(1);

	return size;	
}
int SX127x::read_fifo(void *buf, int size)
{
	uint8_t *data = (uint8_t*)buf;
	cs->write(0);
	systimer->delayus(10);
	spi->txrx(0x00);
	spi->txrx2((uint8_t*)buf, (uint8_t*)buf, size);
	cs->write(1);

	return size;
}


int SX127x::config_DIO(int DIO_index, int source)
{
	if (DIO_index < 0 || DIO_index > 5)
		return -1;
	if (source < 0 || source > 3)
		return -2;

	if (DIO_index < 4)
	{
		uint8_t v = read_reg(0x40);
		int shift = (3 - DIO_index)*2;
		uint8_t mask = ~(0x3 << shift);
		v = (v&mask) | (source << shift);
		write_reg(0x40, v);
	}
	else
	{
		uint8_t v = read_reg(0x41);
		int shift = (7 - DIO_index)*2;
		uint8_t mask = ~(0x3 << shift);
		v = (v&mask) | (source << shift);
		write_reg(0x41, v);
	}

	return 0;
}

int SX127x::set_frequency(float MHz, float crystal /*=32.0f*/)
{
	int freq = MHz / crystal * (1<<19);

	write_reg(0x06, (freq >> 16)&0xff);
	write_reg(0x07, (freq >> 8)&0xff);
	write_reg(0x08, (freq >> 0)&0xff);

	return 0;
}

int SX127x::set_rate(int BW, int CR, int SF)				// BW, CR, SF
{
	if (!(BW == 500000 || BW == 250000 || BW == 125000 || BW == 62500))
		return -1;
	if (CR < 1 || CR > 4)
		return -2;
	if (SF < 7 || SF > 12)
		return -3;

	uint8_t reg = 0x90;	// 500khz, explicit header mode
	if (BW == 250000)
		reg = 0x80;
	if (BW == 12500)
		reg = 0x70;
	if (BW == 62500)
		reg = 0x60;

	reg |= CR << 1;

	write_reg(0x1d, reg);
	write_reg(0x1e, 0x4 | (SF<<4));	// CRC enabled

	return 0;
}

int SX127x::set_tx_power(int dbm)			// and OCP setting
{
	if (dbm < 2 || (dbm > 17 && dbm != 20))
		return -1;

	if (dbm == 20)
	{
		write_reg(0x4d, 0x87);
		write_reg(0x09, 0xff);
	}
	else
	{
		write_reg(0x4d, 0x84);
		write_reg(0x09, 0x80 | (dbm-2));
	}

	write_reg(0x0b, 0x00 | 20);

	return 0;
}

uint8_t SX127x::get_mode()
{
	return read_reg(0x01) & 0x7;
}

int SX127x::get_rssi()
{
	return read_reg(0x1A)-164;
}

void SX127x::self_check()
{
	if (!(read_reg(0x01)&0x80))
	{
		init(spi, cs, txen, rxen);
		printf("self_check reset!\n");
	}
}

void SX127x::set_mode(uint8_t mode)
{
	if (mode == mode_tx)
	{
		if(rxen)
			rxen->write(0);
		if(txen)
			txen->write(1);

		config_DIO(0, 1);
	}

	if (mode == mode_rx || mode == mode_rx_single || mode == mode_CAD)
	{
		if(txen)
			txen->write(0);
		if(rxen)
			rxen->write(1);		
		config_DIO(0, 0);
	}
	uint8_t v = (read_reg(0x01)&0xf8) | (mode&0x7);

	write_reg(0x01, v);
}

int SX127x::write(const void *buf, int block_size)						// write FIFO!
{
	write_reg(0x0d, read_reg(0x0e));
	write_fifo(buf, block_size);
	write_reg(0x22, block_size);
	return 0;
}
int SX127x::read(void *buf, int max_block_size, bool remove /*= true*/)	// read FIFO
{
	int size = read_reg(0x13);
	if (max_block_size < size)
		return -1;
	write_reg(0x0d, read_reg(0x10));
	return read_fifo(buf, size);
}
int SX127x::available()
{
	return read_reg(0x13);
}




SX127xManager::SX127xManager()
{
	fromint = false;
}

SX127xManager::~SX127xManager()
{
	interrupt->set_callback(NULL, NULL);
	timer->set_callback(NULL, NULL);
}

int SX127xManager::init(SX127x *x, HAL::IInterrupt * interrupt, HAL::ITimer *timer)
{
	this->x = x;
	this->interrupt = interrupt;
	this->timer = timer;

	if (interrupt)
		interrupt->set_callback(int_entry, this);
	if (timer)
	{
		timer->set_callback(timer_entry, this);
		timer->set_period(2000);
	}

	last_tx_done_time = -99999;
	tx_interval = 1000;

	return 0;
}

int SX127xManager::write(sx127x_packet p, int priority)
{
	int o = tx_queue[priority].push(p);


	return o;
}

int SX127xManager::flush()
{
	interrupt->disable();
	timer->disable_cb();

	state_maching_go();

	timer->enable_cb();
	interrupt->enable();

	return 0;
}

int SX127xManager::read(sx127x_packet *p)
{
	return rx_queue.pop(p);
}

int SX127xManager::txqueue_space(int priority)		// remaining free space of TX queue
{
	return tx_queue[priority].left();
}
int SX127xManager::txqueue_total(int priority)		// total space of TX queue
{
	return 1;
}
int SX127xManager::rxqueue_count()					// RX available packet count
{
	return rx_queue.count();
}


void SX127xManager::_int(int flags)
{
	//if (!(flags & interrupt_rising))
	//	return;

	timer->disable_cb();
	fromint = true;
	state_maching_go();
	fromint = false;
	timer->enable_cb();
	timer->restart();
}
void SX127xManager::tim()
{
	interrupt->disable();
	state_maching_go();
	interrupt->enable();	
}

bool SX127xManager::ready_for_next_tx()
{
	return mode != mode_tx && mode != mode_prepare_tx && systimer->gettime() > last_tx_done_time + tx_interval;
}

void SX127xManager::state_maching_go()
{
	// check flag
	// rx done: extract packet to rx queue.
	// other: send next packet, if none, enter RX mode

	x->self_check();
	uint8_t flags = x->read_reg(0x12);
	mode = x->get_mode();

	// extract packet to rx queue.
	if (flags&0x40)
	{
		sx127x_packet p;
		p.size = x->read(p.data, sizeof(p.data));
		p.power = x->get_rssi();
		if (rx_queue.push(p) < 0)
		{
			printf("warning:rx buffer overflow\n");
		}

		if (!fromint)
			printf("warning: packet RX DONE not handled in interupt\n");
		x->write_reg(0x12, 0x40);
	}

	// clear TX done flag
	if (flags&0x08)
	{
		x->write_reg(0x12, 0x08);
		last_tx_done_time = systimer->gettime();
		
		if ( tx_interval > 0)
		{
			x->set_mode(mode_rx);
			return;
		}

		if (!fromint)
			printf("warning: packet TX DONE not handled in interupt\n");
	}

	stuck = (mode == mode_prepare_tx) ? stuck+1 : 0;

	if (mode == mode_prepare_tx)
	{
		//if (stuck > 100)
		//	x->set_mode(mode_tx);		// bugfix: sx127x might stuck in mode_prepare_tx
	}

	else if (mode != mode_tx)
	{
		// tx interval passed?
		if (!ready_for_next_tx())
		{
			if (mode != mode_rx)
				x->set_mode(mode_rx);
			return;
		}

		// next TX packet?
		sx127x_packet p = {0};
		for(int i=0; i<2; i++)
		{
			if (tx_queue[i].pop(&p) == 0)
				break;
		}
		if (p.size > 0)
		{
			// yes, go TX
			printf("TX:%d, mode=%d, size=%d\n", int(systimer->gettime()/1000), mode, p.size);
			x->set_mode(mode_standby);
			x->write_reg(0x12, 0x08);
			x->write(p.data, p.size);
			x->set_mode(mode_tx);
			mode = mode_tx;
						
			int64_t timeout = systimer->gettime() + 1000;
			do
			{
				mode = x->get_mode();
			} while (mode != mode_tx && mode != mode_prepare_tx && systimer->gettime() < timeout);
		}
		else
		{
			// no more to go, go RX
			if (mode != mode_rx)
				x->set_mode(mode_rx);
		}
	}	
}

int SX127xManager::cancel_current_packet()
{
	if (!tx_queue[0].count())
		return 0;

	interrupt->disable();
	timer->disable_cb();

	if (x->get_mode() == mode_tx)
		x->set_mode(mode_standby);

	state_maching_go();
	timer->enable_cb();
	interrupt->enable();

	return 0;
}
