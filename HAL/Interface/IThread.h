#pragma once

namespace HAL
{
	typedef void (*thread_entry_t)(void *user_data);

	class IThread
	{
	public:
		virtual ~IThread(){};
		virtual int join() = 0;
		virtual int pause() = 0;
		virtual int resume() = 0;
		virtual int get_id() = 0;
	};

	// platform specified thread creation
	IThread * create_thread(thread_entry_t entry, void *user_data);

	// yield current thread, platform specified
	void yield_thread();
}
