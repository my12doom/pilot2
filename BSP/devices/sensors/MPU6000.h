#pragma once

#include <stdint.h>
#include <Interfaces.h>

namespace sensors
{
	class MPU6000
	{
	public:
		MPU6000(){}
		~MPU6000(){}
		
		int init(HAL::ISPI *SPI, HAL::IGPIO *CS);

		// data[0 ~ 7] :
		// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
		int read(short *data);
		int read_reg(uint8_t reg, void *out, int count);
		int write_reg_core(uint8_t reg, uint8_t data);
		int write_reg(uint8_t reg, uint8_t data);

	protected:
		HAL::ISPI *spi;
		HAL::IGPIO *CS;
	};
}