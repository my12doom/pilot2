#pragma once
#include <stdint.h>

namespace sensors
{

	// assume camera face down and top of camera image aligned to head of airframe.
	// x and y returns "visual angular rate caused by combination of linear(relative to the ground) and angular velocity of body, in radian
	// quality returns estimated image detail level, 0 ~ 1.
	typedef struct
	{
		float x;
		float y;
		float quality;
		int64_t timestamp;
	} flow_data;

	class IFlow
	{
	public:
		// return 0 if new data available, 1 if old data, negative for error.
// 		virtual int read_flow(px4flow_frame *out) = 0;
// 		virtual int read_integral(px4flow_integral_frame *out) = 0;

		// return 0 if new data available, 1 if old data, negative for error.
		// assume camera face down and top of camera image aligned to head of airframe.
		// x and y returns "visual angular rate caused by combination of linear(relative to the ground) and angular velocity of body, in radian
		// quality returns estimated image detail level, 0 ~ 1.
		// x+ points to right side of camera image, y+ points to top side of camera image.
		virtual int read(flow_data *out) = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}