#pragma once

#include <stdint.h>
#include <Interfaces.h>
#include <HAL\Interface\IRangeFinder.h>

namespace sensors
{
	class Sonar :public devices::IRangeFinder
	{	
	private:
		HAL::IGPIO *tx;
		HAL::IGPIO *level;
		float distance;
		bool echo_confirmed;
		int pulse_counter;
		int64_t send_time;
		int64_t first_pulse_time;
		int64_t last_pulse_time;
	public:
		Sonar(){}
		~Sonar(){}
		void echo();	// call this in echo pin interrupt
		virtual int init(HAL::IGPIO *tx, HAL::IGPIO *level);

		// return 0 if new data available, 1 if old data, negative for error.
		// unit: meter.
		// timestamp: unit: milli-second, pass NULL or leave it default to ignore it.
		virtual int read(float *out, int64_t *timestamp = NULL);

		// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
		virtual int trigger();

		// return false if any error/waning
		virtual bool healthy();
	};
}
