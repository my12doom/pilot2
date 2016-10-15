#include "ASysTimer.h"
namespace Android_TIME
{
    ASysTimer::ASysTimer()
	{
		clock_gettime(CLOCK_MONOTONIC,&start);
	}
	ASysTimer::~ASysTimer()
	{

	}
	int64_t ASysTimer::gettime()
	{
		struct timespec tv;
		clock_gettime(CLOCK_MONOTONIC, &tv);
		return (int64_t)((tv.tv_sec-start.tv_sec) * 1000000 + (tv.tv_nsec-start.tv_nsec)/1000);
	}
	void ASysTimer::delayms(float ms)
	{
		usleep(ms*1000);
	}
	void ASysTimer::delayus(float us)
	{
		usleep(us);
	}
	static ASysTimer timer;
}
HAL::ISysTimer *systimer = &Android_TIME::timer;
