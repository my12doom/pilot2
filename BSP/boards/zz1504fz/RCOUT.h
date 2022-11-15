#pragma once

#include <stdint.h>
#include <HAL/Interface/IRCOUT.h>

#define MAX_CHANNEL 4

namespace dev_v2
{
	// channel index starts from 0
	class RCOUT : public HAL::IRCOUT
	{
	public:
		RCOUT();
		~RCOUT(){}
		// total channel count
		virtual int get_channel_count();

		// return num channel written
		// generate an error if index overrun/underrun, and won't update any channel
		// return negative value to indicate an error
		virtual int write(const int16_t *data, int start_channel, int count);

		// return num channel read
		// return any possible read if index overrun
		// return negative value to indicate an error
		virtual int read(int16_t *out, int start_channel, int max_count);
		
	protected:
		int16_t channel_datas[MAX_CHANNEL];
	};
}
