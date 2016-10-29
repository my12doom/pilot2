#pragma once
#include <HAL/Interface/ICriticalSection.h>
#include <pthread.h>
namespace androidUAV
{
	class ACriticalSection : public HAL::ICriticalSection
	{
		public:
			ACriticalSection(){pthread_mutex_init(&mutex,NULL);}
			~ACriticalSection(){pthread_mutex_destroy(&mutex);}
			
			void enter(){pthread_mutex_lock(&mutex);}
			void leave(){pthread_mutex_unlock(&mutex);}
			
			bool try_enter(){return pthread_mutex_trylock(&mutex);}
		protected:
			pthread_mutex_t mutex;//Non-recursive lock
	};
}
