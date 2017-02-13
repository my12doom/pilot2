#pragma once
#include <stdio.h>
#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include <HAL/rk32885.1/ALog.h>

#include <pthread.h>
#include <sched.h>

#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>

#define IOIRQPERIOD 1000
static const char *uioDev = "/dev/uio0";
static const char *uioAddr = "/sys/class/uio/uio0/maps/map0/addr";
static const char *uioSize = "/sys/class/uio/uio0/maps/map0/size";

namespace androidUAV
{
	class AUIOTimer : public HAL::ITimer
	{
		private:
		    int thread;
			pthread_attr_t attr;
			struct sched_param schparam;
			pthread_t tidp;
			pthread_mutex_t mutex;//Non-recursive lock
			HAL::timer_callback cb;
			int period;
			bool exit;
			int policy;
			static void *entry(void *p);
			void run();
			void lock();
			void unlock();
			int uio_fd;
			int eventCount;
			void *user_data;
	    public:
			AUIOTimer(int priority);
			~AUIOTimer();
			virtual void set_period(uint32_t period) ;// micro-second
			virtual void set_callback(HAL::timer_callback cb, void *user_data = 0);
			//SCHED_FIFO,SCHE_RR have a sched_priority valued in the range 1(low) to 99(high)
			//SCHED_FIFO SCHED_RR SCHED_OTHER
			int set_priority(int32_t priority);
	};
};
static int get_thread_policy (pthread_attr_t *attr);
static int get_thread_priority (pthread_attr_t *attr,struct sched_param *param);
static void set_thread_policy (pthread_attr_t *attr,int policy);
static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority);
