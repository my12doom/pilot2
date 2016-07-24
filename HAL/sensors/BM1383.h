#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IBarometer.h>

namespace sensors
{
	class BM1383 : public devices::IBarometer
	{
	public:
		BM1383();
		~BM1383();
		
		int init(HAL::II2C *i2c);	// currently only 0x78(7bit)/0xF0(8bit) address is available

		// IBarometer
		virtual bool healthy();
		virtual int read(devices::baro_data *out);

	protected:
		bool _healthy;
		HAL::II2C *i2c;
	};
}