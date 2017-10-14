#include "NRF24L01.h"

#include <HAL/Interface/ISysTimer.h>

using namespace HAL;


#define TX_ADR_WIDTH    5   //5字节的地址宽度
#define RX_ADR_WIDTH    5   //5字节的地址宽度
uint8_t TX_ADDRESS[] = {0xb0,0x3d,0x12,0x34,0x01}; //????
uint8_t RX_ADDRESS[] = {0xb0,0x3d,0x12,0x34,0x01}; //????
#define TX_PLOAD_WIDTH  32
#define RX_PLOAD_WIDTH  32

//analog register initialization value
uint32_t RegArrFSKAnalog[]={
0x32780504,//for 5.1GHz:33780504 ; 5.8GHz:32780504
0x00AE05C0,
0xD20C80E8,//for 5.1GHz:D38C80E8 ; 5.8GHz:D20C80E8
0x6D7D0D19,//for 5.1GHz:6C7D0D18 ; 5.8GHz:6D7D0D19
0x1B828EE9,//for single carrier:21828EE9
0xA6FF1024,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00000000,
0x00127300,
0x36B48000
};

uint8_t RegArrFSKAnalogReg14[]=
{
0x41,0x20,0x08,0x04,0x81,0x20,0xCF,0xF7,0xFE,0xFF,0xFF
};


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

	spi->set_speed(8000000);
	spi->set_mode(0, 0);

	systimer->delayms(100);
	power_on();
	if (is_bk5811())
	{
		SwitchCFG(1);
		uint8_t WriteArr[12];
		for(int i=0;i<=8;i++)//reverse
		{
			for(int j=0;j<4;j++)
				WriteArr[j]=(RegArrFSKAnalog[i]>>(8*(j) ) )&0xff;

			write_cmd((WRITE_REG|i),&(WriteArr[0]),4);
		}


		for(int i=9;i<=13;i++)
		{
			for(int j=0;j<4;j++)
				WriteArr[j]=(RegArrFSKAnalog[i]>>(8*(3-j) ) )&0xff;

			write_cmd((WRITE_REG|i),&(WriteArr[0]),4);
		}

		write_cmd((WRITE_REG|14),&(RegArrFSKAnalogReg14[0]),11);

		SwitchCFG(0);		
	}

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
	write_reg(EN_RXADDR, 1);
	write_reg(SETUP_AW, TX_ADR_WIDTH-2);
	write_reg(SETUP_RETR, 0);
	write_reg(RF_CH, 95);
	write_reg(RX_PW_P0, RX_PLOAD_WIDTH);
	write_reg(RX_PW_P1, RX_PLOAD_WIDTH);
	write_reg(RF_SETUP, bk5811 ? 0x57 : 0x27);
	write_reg(CONFIG, 0x0e);

	write_reg(7, read_reg(7));	// clear all interrupt

	rf_on(false);

	return 0;
}

int NRF24L01::rf_on(bool rx)
{
	write_reg(CONFIG, (read_reg(CONFIG) & 0xfe) | (rx ? 1 : 0));

	ce->write(true);
	return 0;
}

int NRF24L01::rf_off()
{
	ce->write(false);
	
	return 0;
}

int NRF24L01::write_tx(const uint8_t *data, int count)
{
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
	uint8_t tx[32];
	spi->txrx2(tx, data, count);
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

int NRF24L01::set_tx_address(const uint8_t *address, int address_len/* = 5 */)
{
	write_cmd(TX_ADDR | WRITE_REG, address, address_len);
	write_reg(SETUP_AW, address_len-2);
	
	return 0;
}
int NRF24L01::set_rx_address(int index, const uint8_t *address, int address_len/* = 5 */)
{
	write_cmd((RX_ADDR_P0+index) | WRITE_REG, address, address_len);
	write_reg(SETUP_AW, address_len-2);
	
	return 0;
}
int NRF24L01::enable_rx_address(int index, bool enable)
{
	uint8_t v = read_reg(EN_RXADDR);
	
	if (enable)
		v |= (1<<index);
	else
		v &= ~(1<<index);
	
	write_reg(EN_RXADDR, v);
	
	return 0;
}


// BK5811 helper functions
void NRF24L01::SwitchCFG(bool analog)
{
	uint8_t s = read_reg(7);
	
	if (bool(s&0x80)^analog)
		write_cmd(0x50, 0x53);
}

bool NRF24L01::is_bk5811()
{
	uint8_t s = read_reg(7);
	write_cmd(0x50, 0x53);
	uint8_t s2 = read_reg(7);
	write_cmd(0x50, 0x53);
	
	if ((s&0x80)^(s2&0x80))
		bk5811 = true;
	else
		bk5811 = false;
	return bk5811;
}

bool NRF24L01::bk5811_carrier_test(bool cfg)
{
	if (!is_bk5811())
		return false;
	
	RegArrFSKAnalog[4] = cfg ? 0x21828EE9 : 0x1B828EE9;
	SwitchCFG(1);
	uint8_t WriteArr[12];
	for(int i=0;i<=8;i++)//reverse
	{
		for(int j=0;j<4;j++)
			WriteArr[j]=(RegArrFSKAnalog[i]>>(8*(j) ) )&0xff;

		write_cmd((WRITE_REG|i),&(WriteArr[0]),4);
	}
	SwitchCFG(0);
	
	return true;	
}

}