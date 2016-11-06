#pragma once
#include <stdint.h>
#include <stdlib.h>

namespace devices
{
	enum pixel_type
	{
		L8 = 0,
		YUY2 = 1,
		RGB565 = 2,
		YV12 = 3,
		NV12 = 4,
	};

	typedef struct _frame_format
	{
		int width;
		int height;
		int stride;
		int pixel_type;
	} frame_format;

	typedef struct _time_stamp
	{
		int64_t start;		// start in HAL::ISysTimer units
		int64_t length;		// length in HAL::ISysTimer units
	} timestamp;

	class ICameraCallback
	{
	public:
		virtual void on_frame(timestamp ts) = 0;
		virtual void on_line(timestamp ts) = 0;
		virtual void on_event(int eventcode) = 0;
	};

	class ICamera
	{
	public:
		// get one frame from the camere's internal queue.
		// pp: out pointer
		// timestamp: out pointer
		// only_latest: if true, discard all frame except the latest one.
		// return: 0 if new frame retrived, 1 if no new data, negative values for error.
		virtual int get_frame(uint8_t **pp, timestamp *timestamp=NULL, bool only_latest = false) = 0;

		// get current frame format
		virtual int get_frame_format(frame_format *format) = 0;

		// get available frame format.
		virtual int get_available_frame_format(int index, frame_format *format) = 0;
		int get_available_frame_format_count();

		// set frame format.
		// return 0 if successed, -1 if failed.
		virtual int set_frame_format(const frame_format *format) = 0;

		// set a callback
		virtual int set_callback(ICameraCallback *cb) = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}
