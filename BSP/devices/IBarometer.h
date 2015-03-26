#pragma once
#include <stdint.h>

namespace devices
{
	typedef struct
	{
		float pressure;			// unit: pascal
		float temperature;		// unit: degree Celsius // use NAN if no temperature available.
	} baro_data;

	class IGyro
	{
		// return 0 if new data available, 1 if old data, negative for error.
		int read(baro_data *out);
	};
}