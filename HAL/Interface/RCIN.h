#pragma once

#include <stdint.h>

namespace HAL
{
	// channel index starts from 0
	class RCIN
	{
	public:
		// total channel count
		virtual int get_channel_count();

		// return num channel written to out pointer
		virtual int get_channel_data(int16_t *out, int start_channel, int max_count);

		// return num channel written to out pointer
		virtual int get_channel_update_time(int64_t *out, int start_channel, int max_count);
	};
}
