#pragma once

#include <stdint.h>

namespace HAL 
{
	class ISPI
	{
	public:
		virtual int init() = 0;
		virtual int set_speed(int speed) = 0;		// speed in hz
		virtual int set_mode(int CPOL, int CPHA) = 0;// CPOL: 0 = Idle Low, 1 = Idle High; CPHA: 0 = capture at first edge, 1 = capture at second edge
		virtual	uint8_t txrx(uint8_t data) = 0;
	};
}