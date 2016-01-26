#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

typedef struct struct_sbus_data
{
	uint8_t startbyte;
	int16_t data[16];
	uint8_t flag;
	uint8_t endbyte;
} sbus_data;

namespace sensors
{
	// channel index starts from 0
	class SBusIN : public HAL::IRCIN
	{
	public:
		SBusIN();
		~SBusIN(){}
		bool init(HAL::IUART *port);

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
		void read_uart();
		HAL::IUART *port;
		int64_t last_packet_time;
		sbus_data last_frame;
		int16_t rc_static[2][8];
	};
}
