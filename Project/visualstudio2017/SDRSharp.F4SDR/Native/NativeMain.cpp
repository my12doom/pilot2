#include <Windows.h>
#include <stdio.h>
#include <stdint.h>

#include "device_bulk.h"


CRITICAL_SECTION cs;

typedef int(__stdcall * CSCallback)(const void* ptr, int length_byte);

CSCallback cb = NULL;

float tmp[1024*1024];
float DC[2] = { 0 };

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
	//printf("%d samples", len);
	CSCallback _cb;
	EnterCriticalSection(&cs);

	_cb = cb;
	LeaveCriticalSection(&cs);

	if (_cb)
	{
		float alpha = 1e-5;
		int16_t* p = (int16_t*)buf;
		for (int i = 0; i < len; i++)
		{
			tmp[i] = p[i] / 32768.0f;
			DC[i & 1] = DC[i & 1] * (1 - alpha) + alpha * tmp[i];
			tmp[i] -= DC[i & 1];
			tmp[i] = max(tmp[i], -1.0f);
			tmp[i] = min(tmp[i], 1.0f);
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

int f4sdr_control_io(bool tx, uint8_t request, uint16_t value, uint16_t index, uint8_t *data, uint16_t length)
{
	return device_bulk_control_io(tx, request, value, index, data, length);
}


int f4sdr_tune(int64_t fc)
{
	return device_bulk_tune(fc);
}

int f4sdr_set_gain(uint8_t gain_code)
{
	return device_bulk_gain(gain_code);
}

int f4sdr_set_path(uint8_t path)
{
	return device_bulk_path(path);
}
int f4sdr_enum()
{
	return 0;
}
