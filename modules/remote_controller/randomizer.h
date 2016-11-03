#pragma once

#include <stdint.h>

class randomizer
{
public:
	randomizer()
	{
		reset(0x1234567890345678);
	}
	~randomizer(){}

	void reset(int64_t seed)
	{
		x = seed;
		y = 0xffffffff - seed;
		z = seed >> 32;
		c = seed & 0xffffffff;
	}
	uint32_t next()
	{
		uint64_t t;
		
		x = 1490024343005336237ULL * x + 123456789;
		y ^= y << 21; y ^= y >> 17; y ^= y << 30;			// Do not set y=0!
		t = 4294584393ULL * z + c; c = t >> 32; z = t;		// Avoid z=c=0!

		return (unsigned int)(x>>32) + (unsigned int)y + z;
	}

	uint64_t x, y;
	uint32_t z, c;
};