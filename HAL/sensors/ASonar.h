#pragma once

#include <stdint.h>
#include <HAL/Interface/Interfaces.h>
#include <HAL/Interface/IRangeFinder.h>
#include <HAL/rk32885.1/ALog.h>
#include <Protocol/crc32.h>

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#define SONARARRSIZE 512
typedef struct sonarData
{
	float data;
	int64_t timestamp;
}SonarData;
namespace sensors
{
	class ASonar :public devices::IRangeFinder
	{
	private:
		HAL::IUART *uartSonar;
		int readThreadId;
		int sendThreadID;
		pthread_t tidp;
		pthread_mutex_t mutex;//Non-recursive lock
		bool exit;
		SonarData distanceArray[SONARARRSIZE];
	public:
		ASonar();
		~ASonar();
		virtual int init(HAL::IUART *uart);

		// return 0 if new data available, 1 if old data, negative for error.
		// unit: meter.
		// timestamp: unit: milli-second, pass NULL or leave it default to ignore it.
		virtual int read(float *out, int64_t *timestamp = NULL);

		// trigger messuring manually, this is needed by some types of range finder(sonars e.g.)
		virtual int trigger();

		// return false if any error/waning
		virtual bool healthy();
		
	public:
		static void *read_thread(void *p);
		void readProcess();
		int update_buffer();
	};
}
