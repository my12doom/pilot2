#pragma once

#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>



namespace devices
{
	enum NRF24L01_cmd
	{
		TX_ADDR = 0x10,
		WRITE_REG = 0x20,
		RX_ADDR_P0 = 0x0A,
		RD_RX_PLOAD = 0x61,
		WR_TX_PLOAD = 0xA0,
		FLUSH_TX = 0xE1,
		FLUSH_RX = 0xE2,
		REUSE_TX_PL = 0xE3,
		NOP = 0xFF,
	};

	enum NRF24L01_register
	{
		CONFIG = 0x00,
		EN_AA = 0x01,
		EN_RXADDR = 0x02,
		SETUP_AW = 0x03,
		SETUP_RETR = 0x04,
		RF_CH = 0x05,
		RF_SETUP = 0x06,
		STATUS = 0x07,

		RX_PW_P0 = 0x11,
		RX_PW_P1 = 0x12,

		FIFO_STATUS = 0x17,
	};

class NRF24L01
{
public:
	NRF24L01(){}
	~NRF24L01(){}

	int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce);

	int power_on();		// power up and enter standby-I, should be called only in power down state
	int power_off();	// power down the whole chip.

	virtual int rf_on(bool rx);	// turn on RF, enter RX or TX mode.
	virtual int rf_off();		// turn off RF, in TX mode RF is turned off after the current transmitting packet is sent.

	int write_tx(const uint8_t *data, int count);	// write a packet into FIFO.
	int read_rx(uint8_t *data, int maxcount);		// read a packet from FIFO, return 1 if no new data available.

	int set_tx_address(const uint8_t *address, int address_len = 5);
	int set_rx_address(int index, const uint8_t *address, int address_len = 5);
	int enable_rx_address(int index, bool enable);

	uint8_t read_reg(uint8_t reg);
	void write_reg(uint8_t reg, uint8_t data);
	int write_cmd(uint8_t cmd, const uint8_t *data, int count);
	int read_cmd(uint8_t cmd, uint8_t *data, int count);
	void write_cmd(uint8_t cmd, uint8_t data);
	uint8_t read_cmd(uint8_t cmd);
	bool is_bk5811();
	bool bk5811_carrier_test(bool cfg);				// cfg=true: single carrier test, cfg=false: normal operation

protected:
	HAL::ISPI *spi;
	HAL::IGPIO *cs;
	HAL::IGPIO *ce;

	void SwitchCFG(bool analog);
	bool bk5811;
};
}
