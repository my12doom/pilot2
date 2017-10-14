#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IFlow.h>

namespace sensors
{
	class FoxFlow1 :public IFlow
	{	
	private:
		HAL::IUART *uart;
		float cx, cy;		// fov calibration, default values came from HDF-ov7675 offline calibration.
		int64_t last_valid_packet;

	public:
		FoxFlow1(){}
		~FoxFlow1(){}
		virtual int init(HAL::IUART *uart, float cx = 1.0f / 28.0f * 100 * 3.1415926f / 180, float cy = 1.0f / 28.0f * 100 * 3.1415926f / 180);
		virtual bool healthy();

		// IFlow
		virtual int read(sensors::flow_data *out);
	};
}