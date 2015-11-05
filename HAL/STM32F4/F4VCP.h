#pragma once
#include <HAL/Interface/IUART.h>

namespace STM32F4
{
	class F4VCP:public HAL::IUART
	{
	public:
		F4VCP();
		~F4VCP(){};
		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int flush();
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();
		void destroy();
	};
}
