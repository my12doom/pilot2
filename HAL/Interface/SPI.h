#pragma once

#include <stdint.h>

namespace HAL 
{
	class SPI
	{
	public:
		virtual int init();
		virtual int set_speed();
		virtual int set_mode();
		virtual	int uint8_t txrx();
	};
}