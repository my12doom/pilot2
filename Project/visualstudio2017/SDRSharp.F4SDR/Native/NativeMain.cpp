#include <Windows.h>
#include <stdio.h>
#include <stdint.h>

#include "device_bulk.h"


CRITICAL_SECTION cs;

typedef int(__stdcall * CSCallback)(const void* ptr, int length_byte);

CSCallback cb = NULL;

float tmp[65536];

extern "C" BOOL APIENTRY DllMain(HMODULE /* hModule */, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	if (ul_reason_for_call == DLL_PROCESS_DETACH)
	{
		(void)(lpReserved);
	}
	
	return TRUE;
}

int rx(void *buf, int len, int type)
{
	printf("%d samples", len);
	CSCallback _cb;
	EnterCriticalSection(&cs);

	_cb = cb;
	LeaveCriticalSection(&cs);

	if (_cb)
	{
		int16_t* p = (int16_t*)buf;
		for (int i = 0; i < len; i++)
		{
			tmp[i] = p[i] / 32767.0f;
		}

		_cb(tmp, len * 4);
	}

	return 0;
}


int f4sdr_dll_version()
{
	return 0;
}

int f4sdr_dll_init()
{
	InitializeCriticalSection(&cs);

	return 0;
}

int f4sdr_open()
{
	device_bulk_init(rx);
	return 0;
}

int f4sdr_close()
{
	device_bulk_exit();
	return 0;
}
int f4sdr_cmd()
{
	return 0;
}
int f4sdr_setcb(CSCallback _cb)
{
	EnterCriticalSection(&cs);
	cb = _cb;
	LeaveCriticalSection(&cs);
	return 0;
}

int f4sdr_enum()
{
	return 0;
}
