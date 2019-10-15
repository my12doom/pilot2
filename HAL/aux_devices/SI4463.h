#pragma once

#include <stdint.h>

#include <HAL/Interface/IBlockDevice.h>
#include <HAL/Interface/ISPI.h>
#include <HAL/Interface/IGPIO.h>

class SI4463 : public HAL::IBlockDevice
{
public:
	SI4463();
	~SI4463();

	int init(HAL::ISPI *spi, HAL::IGPIO *cs, HAL::IGPIO *txen = 0, HAL::IGPIO *rxen = 0);
	int write_reg(uint8_t reg, uint8_t v);
	uint8_t read_reg(uint8_t reg);
	int api(uint8_t *in, int in_count, uint8_t *out, int out_count);

	// IBlockDevice
	virtual int write(const void *buf, int block_size);						// write FIFO!
	virtual int read(void *buf, int max_block_size, bool remove = true);	// read FIFO
	virtual int available();

	int set_property(uint16_t property, uint8_t value);
	int get_property(uint16_t property);

	int read_fifo(uint8_t *out, int count);
	int write_fifo(uint8_t *out, int count);
protected:
	int wait_cts(int timeout = 0);
	HAL::ISPI *spi;
	HAL::IGPIO *cs;
	HAL::IGPIO *txen;
	HAL::IGPIO *rxen;

	int default_timeout;
};

