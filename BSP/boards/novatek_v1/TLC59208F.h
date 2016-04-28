#pragma once

#include <stdint.h>
#include <HAL/Interface/IRGBLED.h>
#include <HAL/Interface/II2C.h>

namespace dev_v2
{
	// channel index starts from 0
	class TLC59208F : public devices::IRGBLED
	{
	public:
		TLC59208F();
		~TLC59208F(){}
		
		int init(HAL::II2C *i2c, uint8_t address, uint8_t channel_map[8]);
		virtual int write(float R, float G, float B);
	protected:

		HAL::II2C *i2c;
		uint8_t address;
		uint8_t channel_map[8];

	};
}
