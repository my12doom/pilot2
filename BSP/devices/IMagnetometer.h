#pragma once
#include <stdint.h>

namespace devices
{
	typedef struct
	{
		float x;				// unit: milli-gauss
		float y;
		float z;
		float temperature;		// unit: degree Celsius // use NAN if no temperature available.
	} mag_data;

	class IMagnetometer
	{
		// return 0 if new data available, 1 if old data, negative for error.
		int read(mag_data *out);
	};
}