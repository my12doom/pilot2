#pragma once
#include <HAL/Interface/IUART.h>
#include <modules/utils/fifo2.h>

namespace STM32F1
{
	class F1VCP:public HAL::IUART
	{
	public:
		F1VCP();
		~F1VCP(){};
		virtual int set_baudrate(int baudrate);
		virtual int write(const void *data, int count);
		virtual int flush();
		virtual int read(void *data, int max_count);
		virtual int peak(void *data, int max_count);
		virtual int readline(void *data, int max_count);
		virtual int available();
		void destroy();
			
		FIFO<2048> rx_fifo;
		FIFO<2048> tx_fifo;
	};
}
