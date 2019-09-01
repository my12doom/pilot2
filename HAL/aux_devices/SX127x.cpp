#include "SX127x.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <HAL/Interface/ISysTimer.h>

using namespace HAL;

SX127x::SX127x()
{
	lora_mode = true;
	gfsk_fifo_size = 0;
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

	spi->set_speed(10000000);
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
	_set_mode(mode_sleep);
	//systimer->delayms(10);
	write_reg(0x01, 0x88);		// enable Lora
	//systimer->delayms(10);
	_set_mode(mode_standby);
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
	_set_mode(mode_sleep);
	if (lora_mode)
	{
		write_reg(0x01, 0x88);		// enable Lora
		set_rate(500000, 1, 7);
	}
	else
	{
		write_reg(0x01, 0x08);		// disable Lora
		set_rate(250000);
		write_reg(0x31, 0x40);		// packet mode
		write_reg(0x35, 0x81);		// FIFO threshold size: 1byte.
		write_reg(0x27, 0x92);		// sync on, 3byte sync
		write_reg(0x28, 0x85);
		write_reg(0x29, 0xA3);		// sync word: 0x85A3A3
		write_reg(0x2a, 0xA3);		// sync word: 0x85A3A3
		write_reg(0x0a, 0x39);		// enable BT=0.3 shaping
		write_reg(0x30, 0xD0);		// enable data whitening
		write_reg(0x32, 0xff);		// max RX length: 255 bytes
	}
	
	set_tx_power(20);
	set_frequency(431.5);
	write_reg(0x0e, 0x00);		// disable RSSI smoothing
	_set_mode(mode_standby);

	printf("sx1278 init done\n");
	
	for(int i=0; i<100; i++)
		regs[i] = read_reg(i);

	return 0;
}

int SX127x::set_lora_mode(bool lora)
{
	lora_mode = lora;

	init(spi, cs, txen, rxen);

	return 0;
}

int SX127x::write_reg(uint8_t reg, uint8_t v)
{
	cs->write(0);
	//systimer->delayus(1);
	spi->txrx(0x80 | (reg&0x7f));
	spi->txrx(v);
	cs->write(1);
	return 0;
}

uint8_t SX127x::read_reg(uint8_t reg)
{
	cs->write(0);
	//systimer->delayus(10);
	spi->txrx(reg&0x7f);
	reg = spi->txrx(0);
	cs->write(1);
	return reg;
}

uint8_t dummy[256];
int SX127x::write_fifo(const void *buf, int size)
{
	uint8_t *data = (uint8_t*)buf;
	cs->write(0);
	//systimer->delayus(10);
	spi->txrx(0x80);
	spi->txrx2(data, dummy, size);
	cs->write(1);

	return size;	
}
int SX127x::read_fifo(void *buf, int size)
{
	uint8_t *data = (uint8_t*)buf;
	cs->write(0);
	//systimer->delayus(10);
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

int SX127x::set_rate(int bitrate, float crystal /* = 32.0f */)
{
	float n = crystal * 1e6f / bitrate;
	uint16_t integer = floor(n);
	uint8_t frac = n-integer;

	write_reg(0x2, (integer>>8)&0xff );
	write_reg(0x3, (integer>>0)&0xff );
	write_reg(0x5d, frac);

	// deviation
	float fstep = crystal * 1e6f / (1<<19);
	integer = bitrate/2/fstep;
	write_reg(0x4, (integer>>8)&0xff );
	write_reg(0x5, (integer>>0)&0xff );

	// RX bandwidth
	write_reg(0x12, 0x01);	// currently use a very wide setting.RxBwMant = 00(16), RxBwExp = 1

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
	if (lora_mode)
		return read_reg(0x1A)-164;
	else
		return -read_reg(0x11)/2;
}

void SX127x::self_check()
{
	if (lora_mode && !(read_reg(0x01)&0x80))
	{
		init(spi, cs, txen, rxen);
		printf("self_check reset!\n");
	}
}

void SX127x::_set_mode(uint8_t mode)
{
	if (mode == mode_tx)
	{
		if(rxen)
			rxen->write(0);
		if(txen)
			txen->write(1);

		config_DIO(0, lora_mode ? 1 : 0);
		config_DIO(1, 0);
	}

	if (mode == mode_rx || mode == mode_rx_single || mode == mode_CAD)
	{
		if(txen)
			txen->write(0);
		if(rxen)
			rxen->write(1);		
		config_DIO(0, 0);
		config_DIO(1, 0);
	}
	uint8_t v = (read_reg(0x01)&0xf8) | (mode&0x7);

	write_reg(0x01, v);
}

void SX127x::set_mode(uint8_t mode)
{
	if (lora_mode)
		_set_mode(mode);
	else
	{
		if (mode == mode_tx)
		{
			_set_mode(mode_standby);
			write_reg(0x35, 0x80 | (64 - GFSK_THRESHOLD));		// TX FIFO threshold size
			write_reg(0x36, 0x10 | 0x80);
		}
		else if (mode == mode_rx)
		{
			write_reg(0x32, 0xff);
			write_reg(0x36, 0x10 | 0x40);
			write_reg(0x35, 0x80 | GFSK_THRESHOLD);		// RX FIFO threshold size
			gfsk_fifo_size = 0;
			_set_mode(mode_rx);			
		}
		else
			_set_mode(mode);
	}
}

int SX127x::write(const void *buf, int block_size)						// write FIFO!
{
	if (lora_mode)
	{
		write_reg(0x0d, read_reg(0x0e));
		write_fifo(buf, block_size);
		write_reg(0x22, block_size);
	}
	else
	{
		uint8_t size = block_size;
		write_fifo(&size, 1);
		write_fifo(buf, size > 63 ? 63 : size);

		if (size > 63)
		{
			memcpy(gfsk_fifo, (uint8_t *)buf + 63, size-63);
			gfsk_fifo_size = size-63;
			write_reg(0x35, 0x80 | (64 - GFSK_THRESHOLD));		// TX FIFO threshold size: 1byte.
		}
		else
		{
			write_reg(0x35, 0x80 | (64 - GFSK_THRESHOLD));		// TX FIFO threshold size
			gfsk_fifo_size = 0;
		}
		
	}
	return 0;
}
int SX127x::read(void *buf, int max_block_size, bool remove /*= true*/)	// read FIFO
{
	if (lora_mode)
	{
		int size = available();
		if (max_block_size < size)
			return -1;
		write_reg(0x0d, read_reg(0x10));
		int o = read_fifo(buf, size);
		write_reg(0x12, 0x40);
		return size;
	}
	else
	{
		uint8_t size;
		
		if (gfsk_fifo_size == 0)
		{
			read_fifo(&size, 1);
			if (max_block_size < size)
				return -1;
			read_fifo(buf, size);
		}
		else
		{
			size = gfsk_fifo[0];
			if (max_block_size < size)
			{
				gfsk_fifo_size = 0;
				return -1;
			}
			memcpy(buf, gfsk_fifo+1, gfsk_fifo_size-1);
			read_fifo((uint8_t*)buf+gfsk_fifo_size-1, size - gfsk_fifo_size+1);
			gfsk_fifo_size = 0;
		}
		return size;
	}
}

void SX127x::dio1_int(bool rx_mode)
{
	if (!lora_mode)
	{
		uint8_t reg3f = read_reg(0x3f);
		if (rx_mode)
		{
			if ((reg3f & 0x20) && (sizeof(gfsk_fifo) - gfsk_fifo_size > GFSK_THRESHOLD))
			{
				read_fifo(gfsk_fifo + gfsk_fifo_size, GFSK_THRESHOLD);
				gfsk_fifo_size += GFSK_THRESHOLD;
			}
		}
		else
		{
			if (!(reg3f & 0x20) && gfsk_fifo_size > 0)
			{
				int block_size = gfsk_fifo_size > GFSK_THRESHOLD ? GFSK_THRESHOLD : gfsk_fifo_size;
				write_fifo(gfsk_fifo, block_size);
				memmove(gfsk_fifo, gfsk_fifo + block_size, gfsk_fifo_size - block_size);
				gfsk_fifo_size -= block_size;
			}
		}
	}
}

int SX127x::available()
{
	if (lora_mode)
		return read_reg(0x13);
	else
		return (read_reg(0x31) & 0x3) << 8 | read_reg(0x32);
}

bool SX127x::has_pending_rx()
{
	if (lora_mode)
		return read_reg(0x12)&0x40;
	else
		return read_reg(0x3f)&0x04;
}

void SX127x::clear_tx_done_flag()
{
	if (lora_mode)
		write_reg(0x12, 0x08);
		// nothing to do in fsk mode
}

bool SX127x::tx_done()
{
	if (lora_mode)
		return read_reg(0x12)&0x08;
	else
		return read_reg(0x3f)&0x08;
}


SX127xManager::SX127xManager()
{
	lora_mode = true;
	fromint = false;
	use_aes = false;
}

SX127xManager::~SX127xManager()
{
	interrupt->set_callback(NULL, NULL);
	timer->set_callback(NULL, NULL);
}

int SX127xManager::init(SX127x *x, HAL::IInterrupt * interrupt, HAL::ITimer *timer, HAL::IInterrupt * DIO1)
{
	this->x = x;
	this->interrupt = interrupt;
	this->timer = timer;
	this->DIO1 = DIO1;

	if (interrupt)
		interrupt->set_callback(int_entry, this);
	if (DIO1)
		DIO1->set_callback(int_entry_DIO1, this);
	if (timer)
	{
		timer->set_callback(timer_entry, this);
		timer->set_period(2000);
	}

	last_tx_done_time = -99999;
	tx_interval = 1000;

	set_frequency(433, 433);

	return 0;
}

int SX127xManager::set_frequency(float tx_frequency, float rx_frequency)
{
	this->tx_frequency = tx_frequency;
	this->rx_frequency = rx_frequency;

	return 0;
}

int SX127xManager::write(sx127x_packet p, int priority)
{
	if (p.size <= 0)
		return -1;

	if (use_aes)
	{
		p.size = (p.size + 15)/16*16;
		for(int i=0; i<p.size; i+=16)
			aes.encrypt(p.data+i, p.data+i);
	}

	int o = tx_queue[priority].push(p);

	return o;
}

int SX127xManager::read(sx127x_packet *p)
{
	int64_t t = systimer->gettime();
	int o = rx_queue.pop(p);
	if (o == 0 && use_aes)
	{
		int size = (p->size + 15)/16*16;
		for(int i=0; i<size; i+=16)
			aes.decrypt(p->data+i, p->data+i);
	}
	return o;
}


int SX127xManager::set_aes(uint8_t *key, int keysize)
{
	if (key == NULL || keysize != 32)
	{
		use_aes = false;
		return 0;
	}

	aes.set_key(key, keysize*8);
	use_aes = true;
	return 0;
}


int SX127xManager::flush()
{
	interrupt->disable();
	DIO1->disable();
	timer->disable_cb();

	state_maching_go();

	timer->enable_cb();
	DIO1->enable();
	interrupt->enable();

	return 0;
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
	uint8_t mode = x->get_mode();
	uint8_t reg3f = x->read_reg(0x3f);

	timer->disable_cb();
	DIO1->disable();
	fromint = true;
	state_maching_go();
	fromint = false;
	timer->enable_cb();
	timer->restart();
	DIO1->enable();
}

void SX127xManager::_int_DIO1(int flags)
{
	uint8_t mode = x->get_mode();
	uint8_t reg3f = x->read_reg(0x3f);

	timer->disable_cb();
	interrupt->disable();
	fromint = true;
	state_maching_go();
	fromint = false;
	timer->enable_cb();
	timer->restart();
	interrupt->enable();
}

void SX127xManager::tim()
{
	DIO1->disable();
	interrupt->disable();
	state_maching_go();
	interrupt->enable();	
	DIO1->enable();
}

bool SX127xManager::ready_for_next_tx()
{
	return mode != mode_tx && mode != mode_prepare_tx && systimer->gettime() > last_tx_done_time + tx_interval;
}

int SX127xManager::stuck()
{
	int64_t t = systimer->gettime();
	int tx_timeout = lora_mode ? TX_LORA_STUCK_TIMEOUT : TX_GFSK_STUCK_TIMEOUT;

	if (last_tx_start_time > 0 && t > (last_tx_start_time + tx_timeout))
		return 1;

	return (t > last_rx_time + RX_STUCK_TIMEOUT) ? 2 : 0;
}

int SX127xManager::set_lora_mode(bool lora_mode)
{
	this->lora_mode = lora_mode;

	last_tx_start_time = 0;
	last_rx_time = systimer->gettime();

	interrupt->disable();
	timer->disable_cb();

	int o = x->set_lora_mode(lora_mode);

	timer->enable_cb();
	interrupt->enable();

	return o;
}

void SX127xManager::state_maching_go()
{
	// check flag
	// rx done: extract packet to rx queue.
	// other: send next packet, if none, enter RX mode

	x->self_check();
	mode = x->get_mode();
	if (mode == mode_rx)
		x->dio1_int(true);
	if (mode == mode_tx)
		x->dio1_int(false);



	
	// clear TX done flag
	if (x->tx_done())
	{
		x->clear_tx_done_flag();
		last_tx_done_time = systimer->gettime();
		
		if ( tx_interval > 0)
		{
			x->set_frequency(rx_frequency);
			x->set_mode(mode_rx);

			return;
		}

		if (!fromint)
			printf("warning: packet TX DONE not handled in interupt\n");
		x->set_mode(mode_standby);
		mode = x->get_mode();
	}

	// extract packet to rx queue.
	if (x->has_pending_rx())
	{
		
		sx127x_packet p;
		p.power = x->get_rssi();
		memset(p.data, 0, sizeof(p.data));
		p.size = x->read(p.data, sizeof(p.data));
		last_rx_time = systimer->gettime();
		if (rx_queue.push(p) < 0)
		{
			printf("warning:rx buffer overflow\n");
		}

		if (!fromint)
			printf("warning: packet RX DONE not handled in interupt\n");
		
		//printf("RX:%d bytes, ts=%lld\n",  p.size, systimer->gettime());
	}

	// TX stuck timeout updating
	if (mode == mode_prepare_tx || mode == mode_tx)
		last_tx_start_time = last_tx_start_time ? last_tx_start_time : systimer->gettime();
	else
		last_tx_start_time = 0;

	//if (stuck())
	//	x->set_mode(mode_standby);		// bugfix: sx127x might stuck in mode_prepare_tx

	if (mode == mode_prepare_tx)
	{
	}

	else if (mode != mode_tx)
	{
		// tx interval passed?
		if (!ready_for_next_tx())
		{
			if (mode != mode_rx)
			{
				x->set_frequency(rx_frequency);
				x->set_mode(mode_rx);
			}
			return;
		}

		// TODO : do a CAD detection (and a random sleep) in LORA mode
		if (lora_mode)
		{
			
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
			//printf("TX:%lld, mode=%d, size=%d\n", systimer->gettime(), mode, p.size);
			x->set_mode(mode_standby);
			x->write(p.data, p.size);
			x->set_frequency(tx_frequency);
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
			{
				x->set_frequency(rx_frequency);
				x->set_mode(mode_rx);
			}
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
