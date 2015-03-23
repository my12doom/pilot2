#pragma once
#include <stdint.h>

namespace HAL 
{
	class AnologInput
	{
	public:
		virtual int read() = 0;
	};
}