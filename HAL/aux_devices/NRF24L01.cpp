#include "NRF24L01.h"

#include <HAL/Interface/ISysTimer.h>

using namespace HAL;


#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
const uint8_t TX_ADDRESS[TX_ADR_WIDTH]={0xE7,0xE7,0xE7,0xE7,0xE7}; //发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH]={0xC2,0xC2,0xC2,0xC2,0xC2}; //发送地址
#define TX_PLOAD_WIDTH  32
#define RX_PLOAD_WIDTH  32

namespace devices
{

int NRF24L01::init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce)
{
	this->spi = spi;
	this->cs = cs;
	this->ce = ce;

	cs->set_mode(MODE_OUT_PushPull);
	ce->set_mode(MODE_OUT_PushPull);

	cs->write(true);
	ce->write(false);

	spi->set_speed(10000000);
	spi->set_mode(0, 0);

	systimer->delayms(100);
	power_on();

	write_cmd(FLUSH_TX, NOP);
	write_cmd(FLUSH_RX, NOP);

	uint8_t buf[5] = {0xc2, 0xc2, 0xc2, 0xc2, 0xc2, };
	write_cmd(TX_ADDR | WRITE_REG, buf, 5);
	for(int i=0; i<5; i++)
		buf[i] = 0xff;
	read_cmd(TX_ADDR, buf, 5);

	for(int i=0; i<5; i++)
		if (buf[i] != 0xc2)
			return -1;

	write_cmd(TX_ADDR | WRITE_REG, TX_ADDRESS, TX_ADR_WIDTH);
	write_cmd(RX_ADDR_P0 | WRITE_REG, RX_ADDRESS, RX_ADR_WIDTH);
	write_reg(EN_AA, 0);
	write_reg(EN_RXADDR, 3);
	write_reg(SETUP_RETR, 0);
	write_reg(RF_CH, 24);
	write_reg(RX_PW_P0, RX_PLOAD_WIDTH);
	write_reg(RX_PW_P1, RX_PLOAD_WIDTH);
	write_reg(RF_SETUP, 0x27);
	write_reg(CONFIG, 0x0e);
		
	write_reg(7, read_reg(7));	// clear all interrupt

	rf_on(false);

	return 0;
}

int NRF24L01::rf_on(bool rx)		// power up and enter standby-I, should be called only in power down state
{
	write_reg(CONFIG, (read_reg(CONFIG) & 0xfe) | (rx ? 1 : 0));

	ce->write(true);
	return 0;
}

int NRF24L01::rf_off()
{
	ce->write(false);
}

int NRF24L01::write_tx(const uint8_t *data, int count)
{
	write_reg(STATUS, read_reg(STATUS));
	write_cmd(WR_TX_PLOAD, data, count);

	return 0;
}

int NRF24L01::read_rx(uint8_t *data, int maxcount)
{
	if (maxcount > 32)
		maxcount = 32;

	read_cmd(RD_RX_PLOAD, data, maxcount);

	return maxcount;
}

int NRF24L01::power_on()		// power up and enter standby-I, should be called only in power down state
{
	write_reg(CONFIG, read_reg(CONFIG) | 2);

	return 0;
}
int NRF24L01::power_off()	// power down the whole chip.
{
	write_reg(CONFIG, read_reg(CONFIG) & (~2));
	return 0;
}


int NRF24L01::write_cmd(uint8_t cmd, const uint8_t *data, int count)
{
	cs->write(false);
	spi->txrx(cmd);
	for(int i=0; i<count; i++)
		spi->txrx(data[i]);
	cs->write(true);

	return 0;
}

int NRF24L01::read_cmd(uint8_t cmd, uint8_t *data, int count)
{
	cs->write(false);
	spi->txrx(cmd);
	for(int i=0; i<count; i++)
		data[i] = spi->txrx(0);
	cs->write(true);

	return 0;
}

uint8_t NRF24L01::read_cmd(uint8_t cmd)
{
	uint8_t v;
	read_cmd(cmd, &v, 1);

	return 1;
}
void NRF24L01::write_cmd(uint8_t cmd, uint8_t data)
{
	write_cmd(cmd, &data, 1);
}

uint8_t NRF24L01::read_reg(uint8_t cmd)
{
	uint8_t v;
	read_cmd(cmd & 0x1f, &v, 1);

	return v;
}
void NRF24L01::write_reg(uint8_t cmd, uint8_t data)
{
	write_cmd((cmd & 0x1f) | WRITE_REG, &data, 1);
}

}