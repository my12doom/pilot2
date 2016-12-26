#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IFlow.h>
#include <HAL/Interface/IRangeFinder.h>

namespace sensors
{
	class PX4Flow :public IFlow, devices::IRangeFinder
	{	
	private:
		HAL::II2C *I2C;
		float cx, cy;		// fov calibration, default values came from HDF-ov7675 offline calibration.

	public:
		PX4Flow(){}
		~PX4Flow(){}
		virtual int init(HAL::II2C *I2C, float cx = 1.0f / 28.0f * 100 * 3.1415926f / 180, float cy = 1.0 / 28.0f * 100 * 3.1415926f / 180);
		// return false if any error/waning
		virtual bool healthy();

		// IFlow
		virtual int read_flow(px4flow_frame *out);
		virtual int read_integral(px4flow_integral_frame *out);
		virtual int read(float *x, float *y, float *quality);

		// IRangeFinder
		virtual int trigger(){return 0;}		// ignore trigger() since we don't have that I2C command
		virtual int read(float *out, int64_t *timestamp = NULL);

	};
}