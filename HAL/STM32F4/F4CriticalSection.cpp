#include "F4CriticalSection.h"
#include <stdlib.h>

#define MAX_CS_COUNT 2

using namespace STM32F4;

HAL::ICriticalSection * HAL::create_critical_section()
{
	static F4CriticalSection cs[MAX_CS_COUNT];
	static int used = 0;

	if (used >= MAX_CS_COUNT)
		return NULL;

	return &cs[used++];
}