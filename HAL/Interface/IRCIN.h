#pragma once

#include <stdint.h>

namespace HAL
{
	// channel index starts from 0
	class IRCIN
	{
	public:
		// total channel count
		virtual int get_channel_count() = 0;

		// return num channel written to out pointer
		virtual int get_channel_data(int16_t *out, int start_channel, int max_count) = 0;

		// return num channel written to out pointer
		virtual int get_channel_update_time(int64_t *out, int start_channel, int max_count) = 0;
	
		// statistics functions is mainly for RC calibration purpose.
		virtual int get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count) = 0;
		virtual int reset_statistics() = 0;
	};
}
