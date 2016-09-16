#include "Win32Storage.h"
#include <Windows.h>
#include <stdint.h>
#include "string.h"
using namespace HAL;

namespace SIL_WIN32
{
	Win32Storage::Win32Storage(const wchar_t*filename/* = NULL*/)
	{
		wchar_t _filename[MAX_PATH] = {0};

		if (filename)
			wcscpy_s(_filename, filename);
		else
		{
			GetModuleFileNameW(NULL, _filename, MAX_PATH-1);
			wcscpy((wchar_t*)wcsrchr(_filename, L'\\'), L"\\defaultstorage.bin");
		}

		f = _wfopen(_filename, L"r+b");
		if (!f)
			f = _wfopen(_filename, L"w+b");
	}

	int Win32Storage::init()
	{
		return 0;
	}
	int Win32Storage::erase(int address)
	{
		if (!f || address >= total_size())
			return -1;

		address = address / page_size() * page_size();

		fseek(f, address, SEEK_SET);

		char *tmp = new char[page_size()];
		memset(tmp, 0xff, page_size());
		fwrite(tmp, 1, page_size(), f);
		fflush(f);
		delete[] tmp;

		return 0;
	}
	int Win32Storage::total_size()
	{
		return page_size()*3;
	}
	int Win32Storage::page_size()
	{
		return 128*1024;
	}
	int Win32Storage::write(int address, const void *data, int size)
	{
		if (address <0 || address+size >= total_size() || size <0)
			return -1;

		fseek(f, address, SEEK_SET);
		fwrite(data, 1, size, f);
		fflush(f);

		return size;
	}
	int Win32Storage::read(int address, void *data, int maxsize)
	{
		int size = min(maxsize, total_size() - address);
		if (address <0 || address >= total_size() || size <0)
			return -1;

		fseek(f, address, SEEK_SET);
		fread(data, 1, size, f);
		fflush(f);

		return size;
	}
}

HAL::IStorage *get_default_storage()
{	
	static SIL_WIN32::Win32Storage theDefaultStorage;
	return &theDefaultStorage;
}

HAL::IStorage *get_bootloader_storage()
{
	return NULL;
}
