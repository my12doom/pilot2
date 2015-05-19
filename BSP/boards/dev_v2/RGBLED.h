#pragma once

#include <stdint.h>
#include <HAL/devices/IRGBLED.h>

namespace dev_v2
{
	// channel index starts from 0
	class RGBLED : public devices::IRGBLED
	{
	public:
		RGBLED();
		~RGBLED(){}
		virtual int write(float R, float G, float B);
	protected:
	};
}
