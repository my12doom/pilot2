#pragma once

#include <stdint.h>

template<int step_size, int step_count>
class randomizer
{
public:
	randomizer()
	{
		_pos = 100000000;
		set_seed(0x1234567890345678);
	}

	~randomizer(){}

	void set_seed(int64_t seed)
	{
		this->seed = seed;

		x = seed;
		y = 0xffffffff - seed;
		z = seed >> 32;
		c = seed & 0xffffffff;

		// generate next() table
		for(int i=0; i<step_count; i++)
		{
			next(step_size);
			tbl[i].x = x;
			tbl[i].y = y;
			tbl[i].z = z;
			tbl[i].c = c;
		}
		reset();
	}

	void reset(int pos = 0)
	{
		if (pos <= 1)
		{
			// pos 0
			x = seed;
			y = 0xffffffff - seed;
			z = seed >> 32;
			c = seed & 0xffffffff;
			
			_pos = 1;
			return;
		}

#if 0
		// use table to advance forward
		pos --;
		if (pos >= step_size)
		{
			int tbl_idx = pos/step_size - 1;

			// pos pos/step_size*step_size
			x = tbl[tbl_idx].x;
			y = tbl[tbl_idx].y;
			z = tbl[tbl_idx].z;
			c = tbl[tbl_idx].c;

			pos = pos % step_size;
		}
		else
		{
			// pos 0
			x = seed;
			y = 0xffffffff - seed;
			z = seed >> 32;
			c = seed & 0xffffffff;
		}
		if (pos > 0)
			next(pos);
		
		
#else
		if (pos >= _pos)
		{
			next(pos-_pos);
		}
		else
		{
			// pos 0
			x = seed;
			y = 0xffffffff - seed;
			z = seed >> 32;
			c = seed & 0xffffffff;
			
			next(pos-1);
		}
		_pos = pos;
#endif
	}

	uint32_t next(int count = 1)
	{
		uint64_t t;
		for(int i=0; i<count; i++)
		{
			x = 1490024343005336237ULL * x + 123456789;
			y ^= y << 21; y ^= y >> 17; y ^= y << 30;			// Do not set y=0!
			t = 4294584393ULL * z + c; c = t >> 32; z = t;		// Avoid z=c=0!
			_pos ++;
		}

		return (unsigned int)(x>>32) + (unsigned int)y + z;
	}

protected:
	struct
	{
		uint64_t x, y;
		uint32_t z, c;
	} tbl[step_count];
	uint64_t x, y;
	uint32_t z, c;
	uint64_t seed;
	int _pos;
};
