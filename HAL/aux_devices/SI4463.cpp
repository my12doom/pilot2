#include "SI4463.h"
#include <HAL/Interface/ISysTimer.h>

using namespace HAL;

SI4463::SI4463()
{
	default_timeout = 1000;
}

SI4463::~SI4463()
{

}

int SI4463::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *txen, HAL::IGPIO *rxen)
{
	this->spi = spi;
	this->cs = cs;
	this->txen = txen;
	this->rxen = rxen;

	spi->set_speed(12000000);
	spi->set_mode(0, 0);
	cs->set_mode(MODE_OUT_PushPull);
	cs->write(1);
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

	return 0;
}


int SI4463::write_reg(uint8_t reg, uint8_t v)
{
	cs->write(0);
	spi->txrx(0x80 | (reg&0x7f));
	spi->txrx(v);
	cs->write(1);
	return 0;
}

uint8_t SI4463::read_reg(uint8_t reg)
{
	cs->write(0);
	spi->txrx(reg&0x7f);
	reg = spi->txrx(0);
	cs->write(1);
	return reg;
}
int SI4463::wait_cts(int timeout /* = 0 */)
{
	if (timeout == 0)
		timeout = default_timeout;
	int64_t t = timeout + systimer->gettime();
	uint8_t cts = 0;
	do
	{
		cs->write(0);
		spi->txrx(0x44);
		cts = spi->txrx(0);
		cs->write(1);
	} while (cts != 0xff);

	return cts == 0xff ? 0 : -1;
}

int SI4463::api(uint8_t *in, int in_count, uint8_t *out, int out_count)
{
	if (wait_cts(5000)<0)
		return -1;

	// send command
	cs->write(0);
	for(int i=0; i<in_count; i++)
		spi->txrx(in[i]);
	cs->write(1);

	uint8_t cts = 0;
	do 
	{
		cs->write(0);
		spi->txrx(0x44);
		cts = spi->txrx(0);

		if (cts != 0xff)
		{
			cs->write(1);
			continue;
		}

		for(int i=0; i<out_count; i++)
			out[i] = spi->txrx(0);

		cs->write(1);

		return 0;
	}
	while(1);
}

int SI4463::read_fifo(uint8_t *out, int count)
{
	cs->write(0);
	spi->txrx(0x77);
	for(int i=0; i<count; i++)
		out[i] = spi->txrx(0);
	cs->write(1);

	return count;
}

int SI4463::write_fifo(uint8_t *out, int count)
{
	cs->write(0);
	spi->txrx(0x66);
	for(int i=0; i<count; i++)
		spi->txrx(out[i]);
	cs->write(1);

	return count;
}

int SI4463::set_property(uint16_t property, uint8_t value)
{
	uint8_t inout[5] = {0x11, property >> 8, 1, property&0xff, value};

	return api(inout, 5, 0, 0);
}

int SI4463::get_property(uint16_t property)
{
	uint8_t inout[4] = {0x12, property >> 8, 1, property&0xff};

	int o = api(inout, 4, inout, 1);

	if (o<0)
		return o;

	return inout[0];
}

int SI4463::write(const void *buf, int block_size)						// write FIFO!
{
	return 0;
}
int SI4463::read(void *buf, int max_block_size, bool remove/* = true*/)	// read FIFO
{
	return 0;
}
int SI4463::available()
{
	return 0;
}
