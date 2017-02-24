#include "AUIOTimer.h"
#include <HAL/Interface/ISysTimer.h>

namespace androidUAV
{
	AUIOTimer::AUIOTimer(int priority)
	{
		uio_fd = 0;
		eventCount = 0;
		uio_fd = open(uioDev,O_RDWR|O_SYNC);
		if(uio_fd < 0)
		{
			LOG2("androidUAV:open uio device error %s\n",uioDev);
		}
		pthread_attr_init (&attr);

		policy = get_thread_policy (&attr);
		//Only before pthread_create excuted ,can we set thread parameters
		set_thread_policy(&attr,SCHED_FIFO);
		//update thread pilicy after set thread policy
		policy = get_thread_policy (&attr);
		set_priority(priority);
		pthread_mutex_init(&mutex,NULL);
		thread = pthread_create(&tidp,&attr,entry,(void*)this);
		if(thread < 0)
		{
			//perror
		}
		cb = NULL;
		period = 0;
		exit = false;
	}
	AUIOTimer::~AUIOTimer()
	{
		exit = false;
		pthread_join(thread,0);
		pthread_mutex_destroy(&mutex);
	}
	void AUIOTimer::run()
	{
		int64_t last_call_time = systimer->gettime();
		struct timeval delay;

		while(!exit)
		{
			lock();
			HAL::timer_callback cb = this->cb;
			unlock();
			if(period)
			{
				//int64_t t1 = systimer->gettime();
				int cntTimes = period/IOIRQPERIOD;
				int cntTmp = 0;
				int startEvent = 0;
				int currentEvent = 0;
				int ret = read(uio_fd,&currentEvent,sizeof(currentEvent));
				cntTmp++;
				if(ret < 0)
				{
					LOG2("androidUAV:read uio error\n");
				}
				
			}
			else
				usleep(1000);
			if(cb)
				cb(user_data);
		}
		pthread_exit(NULL);
	}
	void AUIOTimer::lock()
	{
		pthread_mutex_lock(&mutex);
	}
	void AUIOTimer::unlock()
	{
		pthread_mutex_unlock(&mutex);
	}
	void AUIOTimer::set_period(uint32_t period)
	{
		lock();
		this->period = period;
		unlock();
	}
	void AUIOTimer::set_callback(HAL::timer_callback cb, void *user_data/* = 0 */)
	{
		lock();
		this->cb = cb;
		this->user_data = user_data;
		unlock();
	}
	int AUIOTimer::set_priority(int32_t priority)
	{
		//checkout policy ,only SCHED_FIFO SCHED_RR policies have a sched_priority value (1~99)
		if(policy == SCHED_OTHER)
			return -1;
		int32_t priority_min = 0;
		int32_t priority_max = 0;
		priority_min = sched_get_priority_min(policy);
		priority_max = sched_get_priority_max(policy);
		if(priority<=priority_min || priority>=priority_max)
			return -1;
		set_thread_priority(&attr,&schparam,priority);
		return 0;
	}
	void *AUIOTimer::entry(void *p)
	{
		AUIOTimer *_this = (AUIOTimer *)p;
		_this->run();
		return 0;
	}
}
static int get_thread_policy(pthread_attr_t *attr)
{
	int policy;
	int rs = pthread_attr_getschedpolicy(attr, &policy);
	return policy;
}
static int get_thread_priority(pthread_attr_t *attr,struct sched_param *param)
{
	int rs = pthread_attr_getschedparam (attr, param);
	return param->__sched_priority;
}
static void set_thread_policy(pthread_attr_t *attr,int policy)
{
	int rs = pthread_attr_setschedpolicy (attr, policy);
	get_thread_policy (attr);
}
static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority)
{
	param->sched_priority = priority;
	pthread_attr_setschedparam(attr,param);
}

