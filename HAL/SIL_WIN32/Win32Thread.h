#include <HAL/Interface/IThread.h>
#include <Windows.h>

namespace SIL_WIN32
{
	class Win32Thread : public HAL::IThread
	{
	public:

		Win32Thread(HAL::thread_entry_t, void *user_data);
		~Win32Thread();

		int join();
		int pause();
		int resume();
		int get_id();
	protected:
		void *user;
		bool join_called;
		HAL::thread_entry_t _entry;
		HANDLE thread;
		DWORD tid;
		static DWORD WINAPI sentry(LPVOID p){return ((Win32Thread*)p)->entry();}
		DWORD entry();
	};
}