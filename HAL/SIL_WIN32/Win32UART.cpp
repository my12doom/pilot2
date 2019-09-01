#include "Win32UART.h"
#include <stdio.h>
#include <assert.h>

namespace SIL_WIN32
{
	Win32UART::Win32UART()
	:buffer_pos(0)
	,port(INVALID_HANDLE_VALUE)
	{
	}


	int Win32UART::init(int com)
	{
		wchar_t port_name[MAX_PATH];
		wsprintfW(port_name, L"\\\\.\\COM%d", com);

		port = CreateFileW(port_name,
			GENERIC_READ | GENERIC_WRITE,       // Specify mode that open device.
			0,                                  // the devide isn't shared.
			NULL,                               // the object gets a default security.
			OPEN_EXISTING,                      // Specify which action to take on file. 
			FILE_FLAG_NO_BUFFERING,				// default.
			NULL);                              // default.

		set_baudrate(115200);

		return INVALID_HANDLE_VALUE == port ? -1 : 0;
	}

	int Win32UART::init(const char *path)
	{
		buffer_pos = 0;
		port = CreateFileA(path,
			GENERIC_READ | GENERIC_WRITE,       // Specify mode that open device.
			0,                                  // the devide isn't shared.
			NULL,                               // the object gets a default security.
			OPEN_EXISTING,                      // Specify which action to take on file. 
			FILE_FLAG_NO_BUFFERING,				// default.
			NULL);                              // default.
		set_baudrate(115200);
		return INVALID_HANDLE_VALUE == port ? -1 : 0;
	}

	Win32UART::~Win32UART()
	{
		if (INVALID_HANDLE_VALUE != port)
			CloseHandle(port);
		port = INVALID_HANDLE_VALUE;
	}
	void Win32UART::close()
	{
		if (INVALID_HANDLE_VALUE != port)
			CloseHandle(port);
		port = INVALID_HANDLE_VALUE;		
	}
	int Win32UART::set_baudrate(int baudrate)
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;

		DCB config_;
		if (GetCommState(port,&config_) == 0)
			return -1;

		config_.BaudRate = baudrate;    // Specify buad rate of communicaiton.
		config_.StopBits = ONESTOPBIT;    // Specify stopbit of communication.
		config_.Parity = 0;        // Specify parity of communication.
		config_.ByteSize = 8;    // Specify  byte of size of communication.
		if (SetCommState(port,&config_) == 0)
			return -1;

		COMMTIMEOUTS comTimeOut;
		comTimeOut.ReadIntervalTimeout = MAXDWORD;
		comTimeOut.ReadTotalTimeoutMultiplier = 0;
		comTimeOut.ReadTotalTimeoutConstant = 0;
		comTimeOut.WriteTotalTimeoutMultiplier = 3;
		comTimeOut.WriteTotalTimeoutConstant = 0;
		SetCommTimeouts(port,&comTimeOut);

		return 0;
	}
	int Win32UART::peak(void *data, int max_count)
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;

		update_buffer();

		if (max_count >= buffer_pos)
			max_count = buffer_pos;

		if (max_count <= 0)
			return 0;

		memcpy(data, buffer, max_count);

		return max_count;
	}
	int Win32UART::write(const void *data, int count)
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;
		DWORD written = 0;
		WriteFile(port, data, count, &written, NULL);
		return written;
	}
	int Win32UART::flush()
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;
		FlushFileBuffers(port);
		return 0;
	}
	int Win32UART::read(void *data, int max_count)
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;
		update_buffer();

		if (max_count >= buffer_pos)
			max_count = buffer_pos;

		if (max_count <= 0)
			return 0;

		memcpy(data, buffer, max_count);
		memmove(buffer, buffer+max_count, buffer_pos-max_count);
		buffer_pos -= max_count;

		return max_count;
	}
	int Win32UART::readline(void *data, int max_count)
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;
		
		update_buffer();

		int line_end = -1;
		for(int i=0; i<buffer_pos; i++)
		{
			if (buffer[i] == '\n')
			{
				line_end = i+1;
				break;
			}
		}

		if (line_end <= 0 || line_end > max_count)
			return 0;


		memcpy(data, buffer, line_end);
		memmove(buffer, buffer+line_end, buffer_pos-line_end);
		buffer_pos -= line_end;

		return line_end;
	}
	int Win32UART::available()
	{
		if (INVALID_HANDLE_VALUE == port)
			return 0;
		update_buffer();
		return buffer_pos;
	}
	int Win32UART::update_buffer()
	{
		if (INVALID_HANDLE_VALUE == port)
			return -1;		

		DWORD got=0;
		do
		{
			int buffer_left = (sizeof(buffer)/2 - buffer_pos);
			if (buffer_left <= 0)
				return 0;
			BOOL b = ReadFile(port, buffer + buffer_pos, buffer_left, &got, NULL);
			int e = GetLastError();

			char msg[128];
			if (!b && FormatMessageA(FORMAT_MESSAGE_IGNORE_INSERTS | FORMAT_MESSAGE_FROM_SYSTEM,
				NULL, e, 0, msg, sizeof(msg) / sizeof(char), NULL))
			{
				printf(msg);
			}

			buffer_pos += got;
		} while (got);

		return 0;
	}
}
