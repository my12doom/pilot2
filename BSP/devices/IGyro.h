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
		// return 0 if new data available, 1 if old data, negative for error.
		int read(gyro_data *out);

		// return false if any error/waning
		bool healthy();
	};
}