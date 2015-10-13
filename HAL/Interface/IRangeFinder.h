#pragma once
#include <stdint.h>

namespace devices
{
	class IRangeFinder
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
		// unit: meter.
		virtual int read(float *out) = 0;

		// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
		virtual int trigger();

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}
