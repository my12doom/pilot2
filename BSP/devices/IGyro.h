#pragma once
#include <stdint.h>

namespace devices
{
	typedef struct
	{
		float x;				// unit: radian/s
		float y;
		float z;
		float temperature;		// unit: degree Celsius // use NAN if no temperature available.
	} gyro_data;

	class IGyro
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		virtual int read(gyro_data *out) = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}