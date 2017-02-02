#include <stdint.h>
#include <stdio.h>
#include "Win32Timer.h"
#include <HAL/Interface/ISysTimer.h>

using namespace HAL;
namespace SIL_WIN32
{
	Win32Timer::Win32Timer()
	{
		InitializeCriticalSection(&cs);
		thread = CreateThread(NULL, NULL, entry, this, NULL, NULL);
		cb = NULL;
		period = 0;
		exit = false;
	}
	Win32Timer::~Win32Timer()
	{
		exit = true;
		WaitForSingleObject(thread, INFINITE);
		DeleteCriticalSection(&cs);
	}
	void Win32Timer::set_period(uint32_t period)
	{
		EnterCriticalSection(&cs);
		this->period=period;
		LeaveCriticalSection(&cs);
	}
	void Win32Timer::set_callback(timer_callback cb, void *user_data)
	{
		EnterCriticalSection(&cs);
		this->cb=cb;
		this->user_data = user_data;
		LeaveCriticalSection(&cs);
	}

	void Win32Timer::run()
	{
		int64_t last_call_time = systimer->gettime();
		while(!exit)
		{

			EnterCriticalSection(&cs);
			timer_callback cb = this->cb;
			LeaveCriticalSection(&cs);

			if (period)
			{
				int64_t t = systimer->gettime();
				while((t = systimer->gettime()) < last_call_time + period)
				{
					int left = last_call_time + period - t;
					Sleep(left > 1100 ? 1: 0);
				}
				last_call_time = t;
			}
			else
				Sleep(1);
			if (cb)
				cb(user_data);
		}
	}

	void Win32Timer::set_priority(DWORD priority)
	{
		SetThreadPriority(thread, priority);
	}

	DWORD WINAPI Win32Timer::entry(LPVOID p)
	{
		Win32Timer* _this = (Win32Timer*)p;

		_this->run();


		return 0;
	}

}
