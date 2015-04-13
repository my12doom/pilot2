#pragma once

#include <stdint.h>

namespace HAL 
{
	typedef void (*worker_callback)(int parameter);

	class IAsyncWorker
	{
	public:
		// the callback will be called once and only once for each add_work()
		// return 0 on success, negtive value for error.
		virtual int add_work(worker_callback cb, int parameter = 0) = 0;
	};
}