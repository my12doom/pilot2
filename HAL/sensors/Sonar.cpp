#include "Sonar.h"

namespace sensors
{
	int Sonar::init(HAL::IGPIO *tx, HAL::IGPIO *level)
	{
		this->tx = tx;
		this->level = level;
		
		return tx && level ? 0 : -1;
	}
	// return 0 if new data available, 1 if old data, negative for error.
	// unit: meter.
	int Sonar::read(float *out, int64_t *timestamp/* = NULL*/)
	{
		return 0;
	}

	// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
	int Sonar::trigger()
	{
		return 0;
	}
	
	// return false if any error/waning
	bool Sonar::healthy()
	{
		return tx && level ? 0 : -1;
	}
}