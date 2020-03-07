#pragma once
#include <HAL/Interface/ISysTimer.h>
#include <Windows.h>
#include <stdint.h>

namespace SIL_WIN32 
{
	class Win32SysTimer : public HAL::ISysTimer
	{
	public:
		Win32SysTimer();
		~Win32SysTimer();
		virtual int64_t gettime();		// micro-second
		//virtual void delaymsf(float ms);
		//virtual void delayusf(float us);
		virtual void delayms(int ms);
		virtual void delayus(int us);

		LARGE_INTEGER start;
		LARGE_INTEGER fre;
	};
}