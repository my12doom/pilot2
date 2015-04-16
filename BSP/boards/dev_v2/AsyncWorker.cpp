#include "AsyncWorker.h"
#include <BSP\Resources.h>
#include <string.h>
#include <stm32f4xx_exti.h>
#include <HAL\STM32F4\F4Timer.h>

static STM32F4::F4Timer f4TIM5(TIM5);
extern "C" void TIM5_IRQHandler(void)
{
	f4TIM5.call_callback();
}

void workcb()
{
	int left = dev_v2::AsyncWorker::interrupt();
}

int dev_v2::AsyncWorker::work_count = 0;
int dev_v2::AsyncWorker::lock = 0;
dev_v2::async_work dev_v2::AsyncWorker::works[MAX_ASYNC_WORKER] = {0};

dev_v2::AsyncWorker::AsyncWorker()
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	
	//manager.getTimer("log")->set_period(10000);
	//manager.getTimer("log")->set_callback(workcb);

	f4TIM5.set_period(1000);
	f4TIM5.set_callback(workcb);
}

// the callback will be called once and only once for each add_work()
// return 0 on success, negtive value for error.
int dev_v2::AsyncWorker::add_work(HAL::worker_callback cb, int parameter/* = 0*/)
{
	if (work_count >= MAX_ASYNC_WORKER)
		return -1;
	
	NVIC_DisableIRQ(TIM5_IRQn);
	__DSB();
	__ISB();
	
	if (lock)
	{
		NVIC_EnableIRQ(TIM5_IRQn);
		return -1;
	}
	
	async_work work = {cb, parameter};
	works[work_count++] = work;	
	
	NVIC_EnableIRQ(TIM5_IRQn);
	__DSB();
	__ISB();
	//EXTI_GenerateSWInterrupt(EXTI_Line0);
	return 0;
}

int dev_v2::AsyncWorker::interrupt()
{
	lock = 1;
	__DSB();
	__ISB();

	if (work_count <= 0)
	{
		lock = 0;
		return 0;
	}
	
	volatile async_work work = works[0];
	if (work_count > 1)
		memmove(works, works+1, (work_count-1) * sizeof(works[0]));
	work_count --;
	__DSB();
	__ISB();
	lock = 0;
	
	work.cb(work.parameter);
	return work_count;
}
/*
extern "C" void EXTI0_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		int left = dev_v2::AsyncWorker::interrupt();
		EXTI_ClearITPendingBit(EXTI_Line0);
		
		if (left>0)
			EXTI_GenerateSWInterrupt(EXTI_Line0);
	}
}
*/