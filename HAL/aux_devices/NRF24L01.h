#pragma once

#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>



namespace devices
{
class NRF24L01
{
public:
	NRF24L01(){}
	~NRF24L01(){}

	int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *ce);

	int power_on();		// power up and enter standby-I, should be called only in power down state
	int power_off();	// power down the whole chip.

	int rf_on(bool rx);	// turn on RF, enter RX or TX mode.
	int rf_off();		// turn off RF, in TX mode RF is turned off after the current transmitting packet is sent.

	int write_tx(const uint8_t *data, int count);
	int read_rx(uint8_t *data, int maxcount);

	uint8_t read_reg(uint8_t reg);
	void write_reg(uint8_t reg, uint8_t data);
	int write_cmd(uint8_t cmd, const uint8_t *data, int count);
	int read_cmd(uint8_t cmd, uint8_t *data, int count);
	void write_cmd(uint8_t cmd, uint8_t data);
	uint8_t read_cmd(uint8_t cmd);

protected:
	HAL::ISPI *spi;
	HAL::IGPIO *cs;
	HAL::IGPIO *ce;
};
}
