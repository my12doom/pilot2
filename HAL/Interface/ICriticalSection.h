#pragma once
#include <stdint.h>

// this Critical Section is identical to windows "critical section" (with out  first-come, first-serve basis).

namespace HAL
{
	class ICriticalSection
	{
		public:
		// waits (forever) for ownership of the critical section.
		virtual void enter() = 0;

 		// release the ownership of critical section.
		virtual void leave() = 0;

 		// return true if the critical section is successfully entered or current thread already owns the critical section
 		// return false if another thread already owns the critical section.
		virtual bool try_enter() = 0;
	};
	
	// platform specified implementation
	// return NULL if run out of resources
	ICriticalSection * create_critical_section();
}
