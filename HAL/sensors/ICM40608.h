#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IAccelerometer.h>
#include <HAL/Interface/IGyro.h>

namespace sensors
{
	class ICM40608 : public devices::IAccelerometer, public devices::IGyro
	{
	public:
		ICM40608();
		~ICM40608(){}
		
		int init(HAL::ISPI *SPI, HAL::IGPIO *CS);
		int init(HAL::II2C *i2c, uint8_t address);

		// data[0 ~ 7] :
		// raw_temperature, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
		int read(short *data);
		int read_reg(uint8_t reg, void *out, int count);
		int write_reg_core(uint8_t reg, uint8_t data);
		int write_reg(uint8_t reg, uint8_t data);

		// IAccelerometer
		virtual int read(devices::accelerometer_data *out);
		int accelerometer_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez);

		// IGyro
		virtual int read(devices::gyro_data *out);
		int gyro_axis_config(int x, int y, int z, int negtivex, int negtivey, int negtivez);

		// return false if any error/waning
		virtual bool healthy(){return m_healthy;}

	protected:

		int init();

		HAL::ISPI *spi;
		HAL::IGPIO *CS;
		HAL::II2C *i2c;
		uint8_t address;
		bool m_healthy;

		int axis[6];
		int negtive[6];
		short data[7];
	};
}
