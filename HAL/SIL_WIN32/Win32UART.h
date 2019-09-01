#pragma once

#include <stdint.h>
#include <HAL/Interface/IUART.h>
#include <windows.h>

namespace SIL_WIN32
{
	class Win32UART : public HAL::IUART
	{
	public:
		Win32UART();
		int init(int com);
		int init(const char *path);
		~Win32UART();
		int set_baudrate(int baudrate);
		int peak(void *data, int max_count);
		int write(const void *data, int count);
		int flush();
		int read(void *data, int max_count);
		int readline(void *data, int max_count);
		int available();
		void close();

	protected:
		int update_buffer();
		char buffer[20*1024];
		int buffer_pos;	// = 0;
		HANDLE port;
	};
}
