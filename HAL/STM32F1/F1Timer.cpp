#include "F1Timer.h"
#include "F1SysTimer.h"
#include <stdint.h>
#include <stdio.h>
#include <misc.h>
#include <Protocol/common.h>

static STM32F1::F1Timer *tim_tbl[8] = {NULL};

using namespace HAL;
namespace STM32F1
{
	F1Timer::F1Timer(TIM_TypeDef* TIMx)
	{	
		if(TIM1==TIMx)
		{
			IRQn = TIM1_UP_IRQn;
			tim_tbl[1] = this;
		}
		if(TIM2==TIMx)
		{
			IRQn = TIM2_IRQn;
			tim_tbl[2] = this;
		}
		if(TIM3==TIMx)
		{
			IRQn = TIM3_IRQn;
			tim_tbl[3] = this;
		}
		if(TIM4==TIMx)
		{
			IRQn = TIM4_IRQn;
			tim_tbl[4] = this;
		}
#ifdef STM32F10X_HD
		if(TIM5==TIMx)
		{
			IRQn = TIM5_IRQn;
			tim_tbl[5] = this;
		}
		//if(TIM6==TIMx)
		//{
		//	IRQn = TIM6_IRQn;		// TODO: IRQn?
		//	tim_tbl[6] = this;
		//}
		if(TIM7==TIMx)
		{
			IRQn = TIM7_IRQn;
			tim_tbl[7] = this;
		}
#endif		
		this->TIMx=TIMx;
		set_priority(4, 0);
	}
	
	void F1Timer::set_priority(int preemption_priority, int sub_priority/* = 0 */)
	{
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannel = IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preemption_priority;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_priority;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	
	void F1Timer::set_period(uint32_t period)
	{
		if(TIM1==TIMx)
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		if(TIM2==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
		if(TIM3==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
		if(TIM4==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
		if(TIM5==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
		if(TIM6==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
		if(TIM7==TIMx)
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_DeInit(TIMx);
		TIM_InternalClockConfig(TIMx);
		TIM_TimeBaseStructure.TIM_Prescaler= 71;//1Mhz 1us 65536
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
		TIM_TimeBaseStructure.TIM_Period=period-1;
		TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0;
		TIM_TimeBaseInit(TIMx,&TIM_TimeBaseStructure);
		TIM_ClearFlag(TIMx,TIM_FLAG_Update);
		TIM_ARRPreloadConfig(TIMx,DISABLE);
		TIM_ITConfig(TIMx,TIM_IT_Update,ENABLE);
		TIM_Cmd(TIMx,ENABLE);
	}
	void F1Timer::set_callback(timer_callback cb, void *user_data)
	{		
		this->cb=cb;
		this->user_data = user_data;
	}
	
	void F1Timer::restart()
	{
		TIM_Cmd(TIMx,DISABLE);
		TIMx->CNT = 0;
		TIM_ClearITPendingBit(TIMx , TIM_FLAG_Update);		
		TIM_Cmd(TIMx,ENABLE);
	}
	void F1Timer::enable_cb()
	{
		NVIC_EnableIRQ(IRQn);
	}
	
	void F1Timer::disable_cb()
	{
		NVIC_DisableIRQ(IRQn);
	}
	
	void F1Timer::call_callback()
	{
		if (TIMx->SR & TIM_FLAG_Update)
		{
			TIMx->SR = ~TIM_FLAG_Update;
			if(cb)
				cb(user_data);
		}
	}

}
