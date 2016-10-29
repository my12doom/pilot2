#include "ACriticalSection.h"
#include <stdlib.h>

#define MAX_CS_COUNT 2
using namespace androidUAV;
HAL::ICriticalSection* HAL::create_critical_section()
{
	static ACriticalSection cs[MAX_CS_COUNT];
	static int used = 0;
	if(used >= MAX_CS_COUNT)
		return NULL;
	return &cs[used++];
}

