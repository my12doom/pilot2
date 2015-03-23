#pragma once

#include <stdint.h>

namespace HAL 
{
	class SPI
	{
	public:
		virtual int init();
		virtual int set_speed(int speed);	// speed in hz
		virtual int set_mode(int mode);		// see http://en.wikipedia.org/wiki/Serial_Peripheral_Interface_Bus for mode definition
		virtual	uint8_t txrx(uint8_t data);	
	};
}