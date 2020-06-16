#include "ACriticalSection.h"
#include <stdlib.h>

using namespace androidUAV;
HAL::ICriticalSection* HAL::create_critical_section()
{
	return new ACriticalSection();
}

