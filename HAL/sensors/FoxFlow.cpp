#include "FoxFlow.h"
#include <string.h>
#include <Protocol/common.h>
#include <utils/log.h>


namespace sensors
{
	int FoxFlow1::init(HAL::IUART *uart, float cx, float cy)
	{		
		this->cx = cx;
		this->cy = cy;
		this->uart = uart;
		
		last_valid_packet = systimer->gettime();
		uart->set_baudrate(115200);
		
		return 0;
	}

	bool FoxFlow1::healthy()
	{
		return systimer->gettime() - last_valid_packet < 200000;
	}

	static int imin(int a, int b)
	{
		return a>b?a:b;
	}
	int FoxFlow1::read(sensors::flow_data *out)
	{
		uint8_t tmp[200];
		int pkt_size = 14;
		bool got = false;
		
		while (uart->available() > pkt_size)
		{
			// header check
			uart->peak(tmp, 1);
			if (tmp[0] != 0x55)
			{
				uart->read(tmp, 1);
				continue;
			}
			
			// valid packet check
			uart->peak(tmp, pkt_size);
			uint8_t crc = 0;
			for(int i=1; i<pkt_size-1; i++)
				crc += tmp[i];

			if (crc != tmp[pkt_size-1])
			{
				uart->read(tmp, 1);
				continue;
			}
			
			uart->read(tmp, pkt_size);
			short *d = (short*)(tmp+1);
			
			out->x = d[0] * cx;
			out->y = d[1] * cy;
			out->quality = 1;
			out->timestamp = 0;
			got = true;
			
			last_valid_packet = systimer->gettime();
		}
		
		return got ? 0 : 1;
	}
}
