#pragma once

#include <HAL/Interface/ICriticalSection.h>
#include <Windows.h>

namespace SIL_WIN32
{
	class Win32CriticalSection : public HAL::ICriticalSection
	{
	public:

		Win32CriticalSection(){InitializeCriticalSection(&cs);}
		~Win32CriticalSection(){DeleteCriticalSection(&cs);}

		// waits (forever) for ownership of the critical section.
		void enter(){EnterCriticalSection(&cs);}

		// release the ownership of critical section.
		void leave(){LeaveCriticalSection(&cs);}

		// return true if the critical section is successfully entered or current thread already owns the critical section
		// return false if another thread already owns the critical section.
		bool try_enter(){return TryEnterCriticalSection(&cs);}
	protected:
		CRITICAL_SECTION cs;
	};
}
