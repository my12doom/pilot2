#pragma once

#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include <Windows.h>

namespace SIL_WIN32
{
	class Win32Timer:public HAL::ITimer
	{
	private:
		HANDLE thread;
		CRITICAL_SECTION cs;
		HAL::timer_callback cb;
		int period;
		bool exit;
		static DWORD WINAPI entry(LPVOID p);
		void run();
	public:
		Win32Timer();
		~Win32Timer();
		virtual void set_period(uint32_t period);				// micro-second
		virtual void set_callback(HAL::timer_callback cb);	
		void set_priority(DWORD priority);
	};
}