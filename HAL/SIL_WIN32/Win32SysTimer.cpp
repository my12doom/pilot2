#include "Win32SysTimer.h"
#include <Windows.h>
#include <stdint.h>


#pragma comment(lib, "winmm.lib")


namespace SIL_WIN32 
{
	int64_t base = 0;
	int reload;
		
	Win32SysTimer::Win32SysTimer()
	{
		timeBeginPeriod(1);
		QueryPerformanceFrequency(&fre);
		QueryPerformanceCounter(&start);
	}
	
	Win32SysTimer::~Win32SysTimer()
	{
		timeEndPeriod(1);
	}

	
	int64_t Win32SysTimer::gettime()		// micro-second
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);

		return (li.QuadPart - start.QuadPart) * 1000000 / fre.QuadPart;
	}
	
	void Win32SysTimer::delayms(float ms)
	{
		Sleep(ms);
	}
	
	void Win32SysTimer::delayus(float us)
	{
		int64_t s = gettime();
		while(gettime()-s<us)
			Sleep(0);

// 		HANDLE timer; 
// 		LARGE_INTEGER ft; 
// 
// 		ft.QuadPart = -(10*us); // Convert to 100 nanosecond interval, negative value indicates relative time
// 
// 		timer = CreateWaitableTimer(NULL, TRUE, NULL); 
// 		SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0); 
// 		WaitForSingleObject(timer, INFINITE); 
// 		CloseHandle(timer)
	}
	
	static Win32SysTimer timer;
}

HAL::ISysTimer *systimer = &SIL_WIN32::timer;
