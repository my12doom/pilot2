#pragma once
#include <stdint.h>
#include <stdlib.h>

namespace devices
{
	class IRangeFinder
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		// unit: meter.
		// timestamp: unit: milli-second, pass NULL or leave it default to ignore it.
		virtual int read(float *out, int64_t *timestamp = NULL) = 0;

		// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
		virtual int trigger() = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}
