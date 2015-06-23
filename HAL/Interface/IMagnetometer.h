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
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		virtual int read(mag_data *out) = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}