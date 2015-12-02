#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

namespace sensors
{
	// channel index starts from 0
	class PPMIN : public HAL::IRCIN
	{
	public:
		PPMIN();
		~PPMIN(){}
		bool init(HAL::IInterrupt *interrupt);

		// total channel count
		virtual int get_channel_count();

		// return num channel written to out pointer
		virtual int get_channel_data(int16_t *out, int start_channel, int max_count);

		// return num channel written to out pointer
		virtual int get_channel_update_time(int64_t *out, int start_channel, int max_count);
			
		// statistics functions is mainly for RC calibration purpose.
		virtual int get_statistics_data(int16_t *min_out, int16_t *max_out, int start_channel, int max_count);
		virtual int reset_statistics();

	protected:
		static void cb_entry(void *parameter, int flags);
		int handle_ppm(int64_t now);
		int64_t rc_update[10];
		int16_t rc_input[10];
		int16_t rc_static[2][10];

		int64_t last_high_tim;// = -1;
		int ppm_channel_id;// = 0;
		int ppm_channel_count;// = 0;
	};
}
