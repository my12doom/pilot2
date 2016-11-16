#pragma once

#include <HAL/Interface/ICamera.h>
#include <stdint.h>

namespace sensors
{
	class RK3288Camera51 : public devices::ICamera
	{
	public:
		RK3288Camera51();
		~RK3288Camera51();

		int init(int camera_id);

		// get one frame from the camere's internal queue.
		// pp: out pointer
		// timestamp: out pointer
		// only_latest: if true, discard all frame except the latest one.
		// return: 0 if new frame retrived, 1 if no new data, negative values for error.
		virtual int get_frame(uint8_t **pp, devices::timestamp *timestamp=NULL, bool only_latest = false);

		// get current frame format
		virtual int get_frame_format(devices::frame_format *format);

		// get available frame format.
		virtual int get_available_frame_format(int index, devices::frame_format *format);
		virtual int get_available_frame_format_count();

		// set frame format.
		// return 0 if successed, -1 if failed.
		virtual int set_frame_format(const devices::frame_format format);

		// set a callback
		virtual int set_callback(devices::ICameraCallback *cb);

		// return false if any error/waning
		virtual bool healthy() { return _healthy;}

	protected:

		bool _healthy;
	};
}
