#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IBarometer.h>

namespace sensors
{
	class SPL06 : public devices::IBarometer
	{
	public:
		SPL06();
		~SPL06();
		
		int init(HAL::II2C *i2c, int address = 0xEE);	// currently only 0x78(7bit)/0xF0(8bit) address is available

		// IBarometer
		virtual bool healthy();
		virtual int read(devices::baro_data *out);

	protected:
		bool _healthy;
		HAL::II2C *i2c;
		uint8_t address;

		// calibration coeff
		int16_t c0;
		int16_t c1;
		int32_t c00;
		int32_t c10;
		int16_t c01;
		int16_t c11;
		int16_t c20;
		int16_t c21;
		int16_t c30;
	};
}