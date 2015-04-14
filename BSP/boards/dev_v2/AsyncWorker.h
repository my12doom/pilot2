#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>

#define MAX_ASYNC_WORKER 5

namespace dev_v1
{
	typedef struct
	{
		HAL::worker_callback cb;
		int parameter;
	}async_work;
	
	// channel index starts from 0
	class AsyncWorker : public HAL::IAsyncWorker
	{
	public:
		AsyncWorker();
		~AsyncWorker(){}

		// the callback will be called once and only once for each add_work()
		// return 0 on success, negtive value for error.
		virtual int add_work(HAL::worker_callback cb, int parameter = 0);

		static int interrupt();
	private:
		static async_work works[MAX_ASYNC_WORKER];
		static int work_count;
		static int lock;
	};
}
