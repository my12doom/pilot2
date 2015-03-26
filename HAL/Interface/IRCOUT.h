#pragma once

#include <stdint.h>

namespace HAL
{
	// channel index starts from 0
	class IRCOUT
	{
	public:
		// total channel count
		virtual int get_channel_count() = 0;

		// return num channel written
		// generate an error if index overrun/underrun, and won't update any channel
		// return negative value to indicate an error
		virtual int write(const int16_t *data, int start_channel, int count) = 0;
	
		// return num channel read
		// return any possible read if index overrun
		// return negative value to indicate an error
		virtual int read(int16_t *out, int start_channel, int max_count) = 0;

	};
}
