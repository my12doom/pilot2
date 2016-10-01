#pragma once
#include <stdint.h>
#include <HAL/Interface/ITimer.h>
#include <pthread.h>
#include <sched.h>
typedef void* HANDLE;
typedef void* LPVOID;
#define LINUXAPI __attribute__((__stdcall__))
namespace androidUAV
{
    class ATimer : public HAL::ITimer
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
        public:
            ATimer();
            ~ATimer();
            virtual void set_period(uint32_t period) ;// micro-second
            virtual void set_callback(HAL::timer_callback cb);
            //SCHED_FIFO,SCHE_RR have a sched_priority valued in the range 1(low) to 99(high)
            //SCHED_FIFO SCHED_RR SCHED_OTHER
            int set_priority(int32_t priority);
    };
    static int get_thread_policy (pthread_attr_t *attr);
    static int get_thread_priority (pthread_attr_t *attr,struct sched_param *param);
    static void set_thread_policy (pthread_attr_t *attr,int policy);
    static void set_thread_priority(pthread_attr_t *attr,struct sched_param *param,int32_t priority);
}
