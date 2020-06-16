#include "Win32Thread.h"
#include <stdio.h>

using namespace HAL;

// platform specified thread creation
IThread * HAL::create_thread(thread_entry_t entry, void *user_data)
{
	return new SIL_WIN32::Win32Thread(entry, user_data);
}

// yield current thread, platform specified
void HAL::yield_thread()
{
	SwitchToThread();
}

namespace SIL_WIN32
{
Win32Thread::Win32Thread(HAL::thread_entry_t entry, void *user_data)
{
	join_called = false;
	user = user_data;
	_entry = entry;
	thread = CreateThread(0, 0, sentry, this, 0, &tid);
}
Win32Thread::~Win32Thread()
{
	if (!join_called)
		join();	
}

int Win32Thread::join()
{
	join_called = true;
	return WaitForSingleObject(thread, INFINITE);
}

int Win32Thread::pause()
{
	return SuspendThread(thread);
}
int Win32Thread::resume()
{
	return ResumeThread(thread);
}
int Win32Thread::get_id()
{
	return tid;
}

DWORD Win32Thread::entry()
{
	_entry(user);
	return 0;
}

}