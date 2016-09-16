#pragma once
#include <stdint.h>
#include <stdio.h>
#include <wchar.h>
#include <HAL/Interface/IStorage.h>

namespace SIL_WIN32
{
	class Win32Storage:public HAL::IStorage
	{
	public:
		Win32Storage(const wchar_t*filename = NULL);
		~Win32Storage(){};
		virtual int init();
		virtual int total_size();
		virtual int page_size();
		virtual int erase(int address);
		virtual int write(int address, const void *data, int size);
		virtual int read(int address, void *data, int maxsize);

		FILE *f;
	};
}