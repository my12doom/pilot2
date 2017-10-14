#include "VL53L0X.h"
#include <math.h>
#include <Protocol/common.h>

#define VL53L0X_WHO_AM_I							0xC0   // should be 0x40
#define VL53L0X_REG_SYSRANGE_START                  0x00

#define measurement_time 30000

namespace sensors
{
	VL53L0X::VL53L0X()
	{
		last_reading = NAN;
		_healthy = false;
	}

	int VL53L0X::init(HAL::II2C *i2c, uint8_t address /*= 0x52 */)
	{
		this->i2c = i2c;
		this->address = address;

		uint8_t data;
		int o = i2c->read_reg(address, VL53L0X_WHO_AM_I, &data);
		
		if (o<0 || data != 0xEE)
			return -1;

		o = i2c->write_reg(address, VL53L0X_REG_SYSRANGE_START, 1);
		
		_healthy = true;
		
		return _trigger();
	}

	// return 0 if new data available, 1 if old data, negative for error.
	// unit: meter.
	int VL53L0X::read(float *out, int64_t *timestamp/* = NULL*/)
	{
		uint8_t tmp[2];
		int o = i2c->read_reg(address, 0x14, tmp);
		
		if (o<0)
		{
			_healthy = false;
			return -1;
		}
		
		_healthy = true;
		
		if (tmp[0] & 1 )
		{
			i2c->read_regs(address, 0x1E, tmp, 2);
			uint16_t v = (tmp[0] << 8) | tmp[1];
			
			last_reading = (v>0x1f00 || v < 25) ? NAN : (v/1000.0f);
			
			*out = last_reading;
			if (timestamp) 
				*timestamp = trigger_time;
			_trigger();
			return 0;
		}
		
		*out = last_reading;
		if (timestamp)
			*timestamp = trigger_time;		
		
		return 1;
	}

	int VL53L0X::trigger()
	{
		return _healthy;
	}

	// trigger messuring manually, this is needed by some types of range finder(VL53L0Xs e.g.)
	int VL53L0X::_trigger()
	{
		int o = i2c->write_reg(address, VL53L0X_REG_SYSRANGE_START, 1);
		if (o < 0)
		{
			_healthy = false;
			return -1;
		}

		trigger_time = systimer->gettime();
		_healthy = true;

		return 0;
	}

	// return false if any error/waning
	bool VL53L0X::healthy()
	{
		return _healthy;
	}
}
