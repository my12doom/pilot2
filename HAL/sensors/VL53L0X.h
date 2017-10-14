#pragma once

#include <stdint.h>
#include <HAL/Interface/IRangeFinder.h>
#include <HAL/Interface/II2C.h>

namespace sensors
{
	class VL53L0X :public devices::IRangeFinder
	{	
	public:
		VL53L0X();
		~VL53L0X(){}
		virtual int init(HAL::II2C *i2c, uint8_t address = 0x52);

		// return 0 if new data available, 1 if old data, negative for error.
		// unit: meter.
		// timestamp: unit: milli-second, pass NULL or leave it default to ignore it.
		virtual int read(float *out, int64_t *timestamp = NULL);

		// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
		virtual int trigger();

		// return false if any error/waning
		virtual bool healthy();

	private:
		virtual int _trigger();
		bool _healthy;
		HAL::II2C *i2c;
		uint8_t address;
		float last_reading;
		int64_t trigger_time;
	};
}
