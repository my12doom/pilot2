#pragma once
#include <stdint.h>

namespace devices
{
	typedef struct
	{
		float pressure;			// unit: pascal
		float temperature;		// unit: degree Celsius // use NAN if no temperature available.
	} baro_data;

	class IBarometer
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		virtual int read(baro_data *out) = 0;
		
		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}