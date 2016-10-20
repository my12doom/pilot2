#pragma once

#include <HAL/Interface/ICriticalSection.h>

// F4 dummy critical section, no actual locking implementation

namespace STM32F4
{
	class F4CriticalSection : public HAL::ICriticalSection
	{
	public:

		F4CriticalSection(){}
		~F4CriticalSection(){}

		// waits (forever) for ownership of the critical section.
		void enter(){;}

		// release the ownership of critical section.
		void leave(){;}

		// return true if the critical section is successfully entered or current thread already owns the critical section
		// return false if another thread already owns the critical section.
		bool try_enter(){return true;}
	};
}
