#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

#pragma pack(push,1)
typedef struct struct_ebus_data
{
	uint16_t startbytes;
	int16_t data[6];
	uint8_t crc32;
} ebus_data;
#pragma pack(pop)

namespace sensors
{
	// channel index starts from 0
	class EBusIN : public HAL::IRCIN
	{
	public:
		EBusIN();
		~EBusIN(){}
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

		virtual HAL::RCIN_State state();

	protected:
		void read_uart();
		HAL::IUART *port;
		int64_t last_packet_time;
		ebus_data last_frame;
		int16_t rc_static[2][8];
	};
}
