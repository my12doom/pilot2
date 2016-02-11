#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IMagnetometer.h>

namespace sensors
{
	class IST8307A : public devices::IMagnetometer
	{
	public:
		IST8307A();
		~IST8307A(){}

		int init(HAL::II2C *i2c);
		int axis_config(int x, int y, int z, int negtivex, int netgtivey, int negtivez);

		// data[0 ~ 7] :
		// accel_x, accel_y, accel_z, raw_temperature, gyro_x, gyro_y, gyro_z
		int read(short *data);
		int read_reg(uint8_t reg, void *out, int count);
		int write_reg_core(uint8_t reg, uint8_t data);
		int write_reg(uint8_t reg, uint8_t data);

		// IMagnetometer
		virtual int read(devices::mag_data *out);
		virtual bool healthy();

	protected:
		HAL::II2C *i2c;

		int axis[3];
		int negtive[3];
		bool m_healthy;
	};
}
