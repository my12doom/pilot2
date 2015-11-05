#pragma once
#include <stdint.h>
#include <stdlib.h>

namespace devices
{
	enum pixel_type
	{
		L8,
		YUY2,
		RGB565,
	};

	typedef struct _frame_format
	{
		int width;
		int height;
		int stride;
		int pixel_type;
	} frame_format;

	class ICameraCallback
	{
	public:
		virtual void on_frame() = 0;
		virtual void on_line() = 0;
		virtual void on_event(int eventcode) = 0;
	};

	class ICamera
	{
	public:
		virtual int get_frame(void **pp, int64_t *timestamp=NULL) = 0;

		virtual int get_frame_format(frame_format *format) = 0;

		virtual int set_frame_format(const frame_format *format) = 0;
		//
		virtual int set_callback(ICameraCallback *cb) = 0;

		// return false if any error/waning
		virtual bool healthy() = 0;
	};
}
