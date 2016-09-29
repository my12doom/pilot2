#pragma once
#include <HAL/Interface/ISysTimer.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
namespace Android_TIME
{
    class ASysTimer : public HAL::ISysTimer
    {
        public:
            ASysTimer();
            ~ASysTimer();
            virtual int64_t gettime();		// micro-second
            virtual void delayms(float ms);
            virtual void delayus(float us);
        private:
            struct timeval tpend;
            struct timeval timecurrent;
            struct timespec start;
    };
}
