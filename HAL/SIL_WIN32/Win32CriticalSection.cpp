#include "Win32CriticalSection.h"


HAL::ICriticalSection * HAL::create_critical_section()
{
	return new SIL_WIN32::Win32CriticalSection();
}