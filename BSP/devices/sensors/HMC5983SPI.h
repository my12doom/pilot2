#pragma once

#include <stdint.h>
#include <Interfaces.h>

namespace sensors
{
	class HMC5983
	{
	public:
		HMC5983();
		~HMC5983(){}

		int init(HAL::ISPI *SPI, HAL::IGPIO *CS);
		int init(HAL::II2C *i2c);

		// data[0 ~ 7] :
		// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
		int read(short *data);
		int read_reg(uint8_t reg, void *out, int count);
		int write_reg_core(uint8_t reg, uint8_t data);
		int write_reg(uint8_t reg, uint8_t data);

	protected:
		HAL::II2C *i2c;
		HAL::ISPI *spi;
		HAL::IGPIO *CS;
	};
}
